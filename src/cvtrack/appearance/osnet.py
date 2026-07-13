"""OSNet ReID via torchreid (preferred path).

Falls back to a tiny random-projection stub when torchreid or its weights
are unavailable so the rest of the pipeline still runs (ReID just becomes a
no-op).
"""

from __future__ import annotations

import logging
import os
from typing import Optional, Tuple

import numpy as np

from cvtrack.appearance.base import AppearanceExtractor, crop_with_margin, l2_normalize


log = logging.getLogger(__name__)


class OsNetExtractor:
    """torchreid OSNet-based appearance extractor.

    Configurable via:
        model_name : one of ``osnet_x0_25``, ``osnet_x0_5``, ``osnet_x0_75``,
                     ``osnet_x1_0`` (default), etc.
        weights    : optional ``.pth`` file with pretrained ReID weights.  When
                     ``None`` the model is created with random weights, which
                     is still useful for shape / unit tests.
        input_hw   : (height, width) of the network input.  OSNet's native is
                     (256, 128).
        device     : torch device.
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

        try:
            import torch  # noqa: F401
            import torchreid.models.osnet as _osnet  # type: ignore
        except Exception as exc:  # pragma: no cover - import gating
            log.warning("torchreid/OSNet unavailable: %s -- ReID disabled", exc)
            return

        # Construct without pretrained first (always succeeds); we load
        # pretrained weights manually afterwards to avoid the gdown network
        # call that the upstream helper makes.
        try:
            factory = getattr(_osnet, model_name, None)
            if factory is None:
                raise AttributeError(f"unknown model: {model_name}")
            self._model = factory(num_classes=1, pretrained=False, use_gpu=(device != "cpu"))
            self._embedding_dim = self._infer_dim()
        except Exception as exc:
            log.warning("OSNet construction failed: %s -- ReID disabled", exc)
            self._model = None
            return

        # Optionally load user-supplied pretrained weights.
        if weights and os.path.isfile(weights):
            try:
                import torch
                state = torch.load(weights, map_location="cpu")
                # Some checkpoints are wrapped in {"state_dict": ...}.
                if isinstance(state, dict) and "state_dict" in state:
                    state = state["state_dict"]
                missing, unexpected = self._model.load_state_dict(state, strict=False)
                if unexpected:
                    log.info("OSNet: %d unexpected keys in %s", len(unexpected), weights)
                self._loaded_ok = True
                log.info("OSNet: loaded weights from %s (missing=%d)", weights, len(missing))
            except Exception as exc:
                log.warning("OSNet: failed to load weights from %s: %s", weights, exc)
        else:
            if weights:
                log.warning("OSNet: weights path %s not found; using random init", weights)
            else:
                log.info("OSNet: no weights provided; using random init (ReID scores will be meaningless)")

        try:
            self._model.eval()
        except Exception:
            pass

    @property
    def is_available(self) -> bool:
        return self._model is not None

    @property
    def loaded_pretrained(self) -> bool:
        return self._loaded_ok

    @property
    def embedding_dim(self) -> int:
        return int(self._embedding_dim or 0)

    def _infer_dim(self) -> int:
        """Run a single dummy forward to read the embedding dimension."""
        import torch
        self._model.eval()
        with torch.no_grad():
            dummy = torch.zeros(1, 3, self.input_hw[0], self.input_hw[1])
            out = self._model(dummy)
        if isinstance(out, tuple):
            out = out[0]
        return int(out.shape[-1])

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

        resized = cv2.resize(crop, (self.input_hw[1], self.input_hw[0]), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB) if resized.ndim == 3 else resized
        tensor = torch.from_numpy(rgb).float().permute(2, 0, 1).unsqueeze(0) / 255.0
        # ImageNet normalisation -- the default OSNet prep.
        mean = tensor.new_tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
        std = tensor.new_tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)
        tensor = (tensor - mean) / std

        with torch.no_grad():
            out = self._model(tensor)
            if isinstance(out, tuple):
                out = out[0]
        emb = out.squeeze(0).cpu().numpy().astype(np.float64)
        return l2_normalize(emb)