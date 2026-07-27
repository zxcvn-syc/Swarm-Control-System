# scripts/

## 一键启动 / 录屏（联调总指挥：何泓林）

| 脚本 | 用途 |
|------|------|
| `three_links_demo.sh` | 一键 build + source + launch 4 个节点（无录屏） |
| `record_three_links.sh` | 启动 + 录屏（支持 `pseudo` / `ros2bag` / `ffmpeg` 三种模式） |

### 用法

```bash
# 启动（不带录屏）
./scripts/three_links_demo.sh

# 启动（指定视频）
./scripts/three_links_demo.sh --video /abs/path/to/clip.mp4

# 仅构建不启动
./scripts/three_links_demo.sh --dry-run

# 录屏（伪模式：产 .log + 占位 .mp4）
./scripts/record_three_links.sh --out videos/three_links_demo.mp4

# 录屏（ros2 bag 模式：含 6 个核心 topic 的 bag）
./scripts/record_three_links.sh --mode ros2bag
```

详细说明见 `docs/integration/three_link_integration.md`。
