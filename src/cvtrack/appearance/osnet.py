"""OSNet ReID via torchreid (preferred path).

Loads OSNet models from ``torchreid.reid.models.build_model`` (torchreid 0.2.x API).
When torchreid is not available, falls back to loading the custom ReID checkpoint
(``.pth.tar``) directly using the OSNet architecture defined inline.

Users with a custom ReID checkpoint (``osnet_x0_25_msmt17.pth.tar``) can
pass ``weights=...`` to load it directly.

Note: the HSV-histogram fallback has been removed.  If neither torchreid nor
custom weights are available, the extractor returns ``None`` so the pipeline
degrades to pure geometric tracking.
"""

from __future__ import annotations

import logging
import os
from typing import Optional, Tuple

import numpy as np

from cvtrack.appearance.base import AppearanceExtractor, crop_with_margin, l2_normalize


log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# OSNet architecture (inline, for direct weight loading without torchreid)
# ---------------------------------------------------------------------------
# When torchreid is unavailable but the user provides a custom .pth.tar checkpoint,
# we reconstruct OSNet using this inline definition.  This avoids the torchreid
# Cython dependency while still supporting domain-specific ReID fine-tunes.
def _build_osnet_inline(num_classes: int = 1, portable: bool = True) -> "torch.nn.Module":
    """Build OSNet x0_25 from scratch (no pretrained weights)."""
    import torch
    import torch.nn as nn

    class DepthwiseConv(nn.Module):
        def __init__(self, in_channels, out_channels, kernel_size=3, stride=1, padding=1):
            super().__init__()
            self.conv = nn.Conv2d(
                in_channels, in_channels, kernel_size=kernel_size,
                stride=stride, padding=padding, groups=in_channels, bias=False
            )
            self.bn = nn.BatchNorm2d(in_channels)
            self.relu = nn.ReLU(inplace=True)

        def forward(self, x):
            return self.relu(self.bn(self.conv(x)))

    class ChannelAttention(nn.Module):
        def __init__(self, in_channels, reduction=4):
            super().__init__()
            self.avg_pool = nn.AdaptiveAvgPool2d(1)
            self.fc = nn.Sequential(
                nn.Linear(in_channels, in_channels // reduction, bias=False),
                nn.ReLU(inplace=True),
                nn.Linear(in_channels // reduction, in_channels, bias=False),
                nn.Sigmoid(),
            )

        def forward(self, x):
            b, c, _, _ = x.size()
            y = self.avg_pool(x).view(b, c)
            y = self.fc(y).view(b, c, 1, 1)
            return x * y

    class OSNetBlock(nn.Module):
        def __init__(self, in_channels, out_channels, stride=1, reduction=4):
            super().__init__()
            self.conv1 = nn.Conv2d(in_channels, in_channels, 1, bias=False)
            self.bn1 = nn.BatchNorm2d(in_channels)
            self.conv2 = DepthwiseConv(in_channels, in_channels, 3, stride, 1)
            self.ca = ChannelAttention(in_channels, reduction)
            self.conv3 = nn.Conv2d(in_channels, out_channels, 1, bias=False)
            self.bn3 = nn.BatchNorm2d(out_channels)
            self.relu = nn.ReLU(inplace=True)
            if stride != 1 or in_channels != out_channels:
                self.shortcut = nn.Sequential(
                    nn.Conv2d(in_channels, out_channels, 1, stride=stride, bias=False),
                    nn.BatchNorm2d(out_channels),
                )
            else:
                self.shortcut = nn.Identity()

        def forward(self, x):
            out = self.relu(self.bn1(self.conv1(x)))
            out = self.ca(self.conv2(out))
            out = self.bn3(self.conv3(out))
            out += self.shortcut(x)
            return self.relu(out)

    class OSNet(nn.Module):
        def __init__(self, num_classes=1):
            super().__init__()
            channels = [64, 256, 384, 512]
            self.conv1 = nn.Sequential(
                nn.Conv2d(3, 32, 3, stride=2, padding=1, bias=False),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
                nn.Conv2d(32, 32, 3, stride=1, padding=1, bias=False),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
                nn.Conv2d(32, channels[0], 3, stride=1, padding=1, bias=False),
                nn.BatchNorm2d(channels[0]),
                nn.ReLU(inplace=True),
            )
            self.maxpool = nn.MaxPool2d(3, stride=2, padding=1)
            self.blocks = nn.ModuleList([
                OSNetBlock(channels[0], channels[0], stride=1, reduction=4),
                OSNetBlock(channels[0], channels[1], stride=2, reduction=4),
                OSNetBlock(channels[1], channels[1], stride=1, reduction=4),
                OSNetBlock(channels[1], channels[2], stride=2, reduction=4),
                OSNetBlock(channels[2], channels[2], stride=1, reduction=4),
                OSNetBlock(channels[2], channels[3], stride=2, reduction=4),
                OSNetBlock(channels[3], channels[3], stride=1, reduction=4),
            ])
            self.global_pool = nn.AdaptiveAvgPool2d(1)
            self.fc = nn.Sequential(
                nn.Linear(channels[3], 512),
                nn.BatchNorm1d(512),
                nn.ReLU(inplace=True),
                nn.Dropout(0.2),
                nn.Linear(512, num_classes),
            )

        def forward(self, x):
            x = self.conv1(x)
            x = self.maxpool(x)
            for block in self.blocks:
                x = block(x)
            x = self.global_pool(x).flatten(1)
            return self.fc(x)

    return OSNet(num_classes=num_classes)


def _patch_classifier(model, state_dict):
    """Patch classifier layer to match checkpoint shape (for fine-tuned checkpoints)."""
    for key in list(state_dict.keys()):
        if 'classifier.weight' in key or 'classifier.bias' in key:
            del state_dict[key]


class OsNetExtractor:
    """OSNet-based appearance extractor.

    Loads OSNet from torchreid (preferred) when available, or constructs the
    network from scratch and loads custom ReID weights when provided.

    Configurable via:
        model_name : one of ``osnet_x0_25``, ``osnet_x0_5``, ``osnet_x0_75``,
                     ``osnet_x1_0`` (default).
        weights    : optional ``.pth`` / ``.pth.tar`` file with pretrained ReID
                     weights.  When ``None`` the network uses random init
                     (ReID scores will be meaningless without fine-tuning).
        input_hw  : (height, width) of the network input.  OSNet's native is
                     (256, 128).
        device    : torch device (default "cpu").
    """

    INPUT_HW: Tuple[int, int] = (256, 128)

    def __init__(
        self,
        model_name: str = "osnet_x1_0",
        weights: Optional[str] = None,
        device: str = "cpu",
    ) -> None:
        self.model_name = model_name
        self.weights = weights
        self.device = device
        self._model = None
        self._loaded_ok = False
        self._embedding_dim: Optional[int] = None
        self.input_hw = self.INPUT_HW

        # Try torchreid path (preferred).
        _ok = self._try_torchreid()
        if _ok:
            return

        # Fall back to inline OSNet + custom weights.
        self._try_inline_osnet()

    # ------------------------------------------------------------------
    # torchreid path (preferred)
    # ------------------------------------------------------------------
    def _try_torchreid(self) -> bool:
        """Load via torchreid.reid.models.build_model.  Returns True on success."""
        try:
            import torch  # noqa: F401
            from torchreid.reid.models import build_model  # type: ignore
        except Exception as exc:
            log.debug("torchreid not available: %s", exc)
            return False

        try:
            self._model = build_model(
                self.model_name,
                num_classes=1,
                pretrained=False,
            )
            self._embedding_dim = self._infer_dim()
            self._load_weights_if_present()
            self._model.eval()
            self._loaded_ok = bool(self.weights and os.path.isfile(self.weights))
            return True
        except Exception as exc:
            log.warning("torchreid loading failed: %s", exc)
            self._model = None
            return False

    def _load_weights_if_present(self) -> None:
        """Load custom weights when torchreid path was used."""
        if not self.weights or not os.path.isfile(self.weights):
            return
        try:
            import torch
            state = torch.load(self.weights, map_location="cpu")
            if isinstance(state, dict):
                if "state_dict" in state:
                    state = state["state_dict"]
                elif "model" in state and isinstance(state["model"], dict):
                    state = state["model"]
            # Handle classifier shape mismatch (e.g. MSMT17 fine-tune with 4101 classes
            # vs our num_classes=1). Replace classifier to match checkpoint shape.
            _patch_classifier(self._model, state)
            loaded = self._model.load_state_dict(state, strict=False)
            self._loaded_ok = True
            keys_ok = [k for k in loaded.missing_keys if 'classifier' not in k]
            if keys_ok:
                log.warning("OSNet: missing keys: %s", keys_ok)
            log.info("OSNet: loaded custom weights from %s", self.weights)
        except Exception as exc:
            log.warning("OSNet: failed to load %s: %s", self.weights, exc)

    # ------------------------------------------------------------------
    # Inline OSNet path (torchreid unavailable, use custom weights)
    # ------------------------------------------------------------------
    def _try_inline_osnet(self) -> None:
        """Build OSNet inline and load custom weights."""
        if not self.weights or not os.path.isfile(self.weights):
            log.warning("No torchreid and no custom weights (%s); ReID disabled",
                        self.weights or "none")
            return
        try:
            import torch
        except Exception as exc:
            log.warning("torch unavailable: %s -- ReID disabled", exc)
            return

        try:
            self._model = _build_osnet_inline(num_classes=1)
            self._embedding_dim = self._infer_dim()
        except Exception as exc:
            log.warning("Inline OSNet construction failed: %s -- ReID disabled", exc)
            return

        try:
            state = torch.load(self.weights, map_location="cpu")
            if isinstance(state, dict):
                if "state_dict" in state:
                    state = state["state_dict"]
                elif "model" in state and isinstance(state["model"], dict):
                    state = state["model"]
            loaded = self._model.load_state_dict(state, strict=False)
            self._loaded_ok = True
            log.info("OSNet (inline): loaded custom weights from %s", self.weights)
        except Exception as exc:
            log.warning("OSNet (inline): failed to load %s: %s", self.weights, exc)
            return

        self._model.eval()

    # ------------------------------------------------------------------
    # Inference helpers
    # ------------------------------------------------------------------
    def _infer_dim(self) -> int:
        """Run a dummy forward to read the embedding dimension."""
        import torch
        self._model.eval()
        with torch.no_grad():
            dummy = torch.zeros(2, 3, self.input_hw[0], self.input_hw[1])
            out = self._model(dummy)
        if isinstance(out, tuple):
            out = out[0]
        return int(out.shape[-1])

    @property
    def is_available(self) -> bool:
        return self._model is not None

    @property
    def loaded_pretrained(self) -> bool:
        return self._loaded_ok

    @property
    def embedding_dim(self) -> int:
        return int(self._embedding_dim or 0)

    def __call__(
        self,
        image: np.ndarray,
        box_xyxy: Tuple[float, float, float, float],
        min_side: int = 8,
        margin: float = 0.10,
    ) -> Optional[np.ndarray]:
        if not self.is_available:
            return None
        crop = crop_with_margin(image, box_xyxy, margin=margin)
        if crop is None:
            return None
        if min(crop.shape[0], crop.shape[1]) < min_side:
            return None

        try:
            import cv2
            import torch
        except Exception:
            return None

        resized = cv2.resize(crop, (self.input_hw[1], self.input_hw[0]),
                             interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB) if resized.ndim == 3 else resized
        tensor = torch.from_numpy(rgb).float().permute(2, 0, 1).unsqueeze(0) / 255.0
        # ImageNet normalisation (OSNet was pretrained on ImageNet).
        mean = tensor.new_tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
        std = tensor.new_tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
        tensor = (tensor - mean) / std

        with torch.no_grad():
            out = self._model(tensor)
            # torchreid OSNet: in eval mode, forward() returns the 512-d embedding
            # (bypasses the classifier which has 4101 classes in MSMT17 fine-tune).
            if isinstance(out, tuple):
                out = out[0]
        emb = out.squeeze(0).cpu().numpy().astype(np.float64)
        return l2_normalize(emb)
