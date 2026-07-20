# MIGRATION

本仓库经历过一次结构性整顿，文档化的所有变更都已经落在工作区里。但有几件事需要在
**远端 GitHub 仓库**和**本地 git 历史**层面做对，开发环境之外的协作者才看得到。

如果你不打算把变更推上去，可以忽略这份文件。

## 1. 改 GitHub 仓库名（ROS → cvtrack）

仓库根目录和 Python package 都已经统一叫 `cvtrack`，但 GitHub 远端的名字
（`<owner>/ROS`）仍需要手动改。

1. 在 GitHub 上进入仓库页面 → **Settings** → 滚动到最底部 **Danger Zone** →
   **Rename this repository** → 改成 `<owner>/cvtrack`。
2. 改完后通知协作者更新 remote：

```bash
git remote set-url origin https://github.com/<owner>/cvtrack.git
```

GitHub 会自动把旧 URL 重定向到新 URL，但显式更新更干净。

## 2. 从 git 历史中移除已入库的大文件

`weights/run_*/` 下的所有 CSV/JSON 和 `weights/COMPARE_REPORT.md`
（合计 ~4.5 MB 的运行产物）已经从工作区删除，新 `.gitignore` 也会阻止它们再回来。
但它们仍留在 git 历史里 —— 任何 `git clone` 的人仍然会拉下这些字节。

把它们从历史里清掉的标准做法是用 `git-filter-repo`：

```bash
# 安装（一次性）
pip install git-filter-repo

# 在仓库根目录执行
git filter-repo --invert-paths \
    --path-glob 'weights/**' \
    --path 'sample_gt.csv' \
    --path 'postprocess.py'
```

这会把所有 `weights/` 子目录、`sample_gt.csv` 和旧的根目录 `postprocess.py`
从**每一个 commit** 中剥离。然后强制推送：

```bash
git remote add origin https://github.com/<owner>/cvtrack.git   # 如果还没加
git push origin --force --all
git push origin --force --tags
```

> ⚠️ `git push --force` 会**重写远端历史**。在执行之前确保所有协作者都已推送
> 自己的本地变更，并通知他们 clone 后必须重新拉一次（普通的 `git pull` 不会
> 处理被重写的历史）。

### 替代方案（不推荐）

如果不能用 `git-filter-repo`，可以用内置的 `git filter-branch`，但速度慢 100×，
且更容易出错。`BFG Repo-Cleaner` 是另一个选项（Java 工具）。

## 3. （可选）把旧名 `cv_tracking_demo` 从历史中清掉

工作区里已经没有 `cv_tracking_demo` 字符串残留（`rg cv_tracking_demo .` 应为 0 命中），
但历史里还有。如果你想做一次"彻底改名"：

```bash
# 先在工作区把所有仍存在的旧名字替换为新名字（前面已经做过了，这里只是示例）
# rg -l 'cv_tracking_demo' | xargs sed -i 's/cv_tracking_demo/cvtrack/g'

git filter-repo --replace-text expressions.txt
```

`expressions.txt` 格式：

```
cv_tracking_demo==>cvtrack
/home/hhh/Downloads/cv_tracking_demo==>/path/to/cvtrack
```

这一步是**可选**的：MIGRATION 的核心目标是清大文件，名字只是个卫生问题，
新的 README 和 MOG2 警告已经在工作区层面对用户可见。

## 4. 校验

完成 1 + 2 之后：

```bash
# 远端大小应该显著缩水
git clone https://github.com/<owner>/cvtrack.git cvtrack-verify
du -sh cvtrack-verify/.git

# 工作区干净
cd cvtrack-verify
git log --oneline -5
git status
ls weights/                # 应该只有 .gitkeep
rg cv_tracking_demo .      # 应该 0 命中
```

`.git` 目录应该从 ~34 MB 降到 < 5 MB。

## 5. 本次变更清单（已在工作区中应用）

| 类型     | 路径                                               | 说明                                     |
|----------|----------------------------------------------------|------------------------------------------|
| 新增     | `LICENSE`                                          | Apache-2.0 全文                          |
| 新增     | `weights/.gitkeep`                                 | 让 `weights/` 目录在 git 中保留         |
| 修改     | `.gitignore`                                       | 覆盖所有运行产物（csv/json/mp4/weights） |
| 修改     | `README.md`                                        | 已翻译为中文；MOG2 警告；仓库名说明      |
| 修改     | `pyproject.toml`                                   | license/authors/Source URL 一致化       |
| 修改     | `configs/drone.yaml`                               | 权重文件名注释修正（`.pt` → `.pth.tar`） |
| 修改     | `src/cvtrack/tracker/deepsort.py`                  | 去除 `DEEPSORT_MAHALANOBIS_GATE` 重复定义 |
| 修改     | `src/cvtrack/pipeline.py`                          | 默认 `--out-dir` 和权重回退改本地约定    |
| 修改     | `scripts/visdrone_compare.py`                      | COCO 权重回退改本地约定                  |
| 修改     | `scripts/run_cvtrack.sh`                           | `LOCAL_LIB` 默认值改为可空               |
| 修改     | `scripts/VISDRONE_COMPARE_README.md`               | `cd /path/to/cv_tracking_demo` → `cvtrack` |
| 删除     | `weights/**`（整目录）                             | 所有运行产物 CSV/JSON                    |
| 删除     | `sample_gt.csv`                                    | 一次性 GT 示例                           |
| 删除     | `postprocess.py`（根目录旧入口脚本）                | 已被 `python -m cvtrack.postprocess` 取代 |

## 6. 提交建议

工作区现在是干净状态。本地 commit + push 推荐分两个 commit：

```bash
git add -A
git commit -m "chore: clean up repo, translate README, add LICENSE

- delete weights/ run outputs (~4.5 MB) and root-level sample_gt.csv / postprocess.py
- expand .gitignore to cover all run artefacts
- add Apache-2.0 LICENSE
- translate README to Chinese, surface MOG2-by-default warning
- rename references cv_tracking_demo -> cvtrack, drop hard-coded /home/hhh paths
- fix duplicated DEEPSORT_MAHALANOBIS_GATE definition in deepsort.py
- fix drone.yaml weight filename (.pt -> .pth.tar)"

# 如果你执行了步骤 2 的 filter-repo，第二个 commit 自然不会出现 —— 整段历史已经被重写。
git push origin main
```
