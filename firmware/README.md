# 外部STM32工程修复补丁

完整工程位于 `/mnt/data/LeArm_APP_rtos`，本目录仅维护需要同步回原工程的修复，不包含厂商完整源码、静态库或构建产物。

`patches/health-startup.patch` 修复健康模块漏初始化：main在RTOS启动前调用 `robot_health_init`；get_snapshot返回成功/失败，未初始化或NULL时失败；SerialTask只发送有效快照，并清零局部状态作防御。增加一组未初始化路径测试。保留原安全状态机、STOP/BOOT和运动控制逻辑。

2026-11-12当前本机外部源码**已修复**，不需要再次应用。以后重新下载相同旧工程时，Ubuntu仓库根目录执行：

```bash
patch --binary --forward --dry-run -p1 -d /mnt/data/LeArm_APP_rtos < firmware/patches/health-startup.patch
# 仅在上一步所有文件均通过、且原工程尚未修复时执行：
patch --binary --forward -p1 -d /mnt/data/LeArm_APP_rtos < firmware/patches/health-startup.patch
bash /mnt/data/LeArm_APP_rtos/tests/run_health_tests.sh
```

patch保留CubeMX main.c的CRLF字节，必须保留 `--binary`；不要求改原文件行尾。补丁在相同未修改原文件上dry-run和实际应用均已验证。其他版本工程应人工合并，不强制套用。

GCC构建工具会拒绝main缺少健康初始化，链接后进一步检查main中的调用顺序。旧版54,624字节产物有此缺陷，使用[当前修复固件与IAP说明](../docs/STM32本地编译与IAP.md)中的54,752字节版本。
