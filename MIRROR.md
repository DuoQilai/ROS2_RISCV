# Course mirror

`master` tracks the course history from https://gitee.com/yunxiangluo/ROS2_RISCV and may also contain GitHub-only course catalogs awaiting an upstream contribution.
Use `master` for course document URLs. Submit GitHub course changes through pull requests.

The default branch, `main`, holds mirror automation and is not the current course edition.
The **Sync Gitee mirror** workflow runs hourly at minute 17 and supports **Run workflow** in GitHub Actions. Scheduled runs may be delayed by GitHub.

Synchronization merges upstream changes into the current GitHub `master`, fast-forwarding when possible and otherwise creating a signed-off merge commit. GitHub-only catalog files are preserved by the merge. If upstream is already included, no commit is created. Conflicts abort the merge without pushing; fetch failures and concurrent remote updates also stop the run without overwriting remote commits. Review failed runs in Actions before retrying.

Deploy this workflow on `main` before merging GitHub-only catalogs into `master`. The catalogs can be contributed to Gitee with a later course update. Identical files on both hosts do not by themselves restore identical commit history; keep merge synchronization unless a separate history-alignment change has been reviewed.

This sync does not deploy the documentation website. Rebuild the frontend to publish updated course documents.
