# Course mirror

`master` mirrors the course history from https://gitee.com/yunxiangluo/ROS2_RISCV.
Use `master` for course document URLs. Do not edit it directly.

The default branch, `main`, holds mirror automation and is not the current course edition.
The **Sync Gitee mirror** workflow runs hourly at minute 17 and supports **Run workflow** in GitHub Actions. Scheduled runs may be delayed by GitHub.

Synchronization only fast-forwards `master`. Network failures or divergent history fail the run without overwriting the mirror. Review failed runs in Actions before retrying.

This sync does not deploy the documentation website. Rebuild the frontend to publish updated course documents.
