# STM32 App 源码和历史补丁

当前唯一维护的完整 App 位于 **[LeArm_APP_rtos](LeArm_APP_rtos/README.md)**。
Keil 打开 `LeArm_APP_rtos/MDK-ARM/LeArm.uvprojx`，目标为 `LeArm`。
来源下载目录 `/mnt/data/LeArm_APP_rtos` 保留，不再对其继续开发。

2026-11-12已修复健康与安全两处漏初始化：健康在内核初始化前完成；安全在内核Ready状态、任何业务任务创建前完成，失败停止启动。快照成功标志、防御清零、FAULT锁存和STOP/BOOT授权检查均保留。软件测试和完整ARM GCC链接通过，Keil/实机未执行。

Ubuntu仓库根目录：

```bash
python3 scripts/check_stm32_project.py
bash firmware/LeArm_APP_rtos/tests/run_safety_tests.sh
bash firmware/LeArm_APP_rtos/tests/run_health_tests.sh
python3 scripts/build_stm32.py --output build/stm32-startup-fixed
```

工具链不在PATH时附加 `--toolchain-bin /实际工具链/bin`。
新GCC检查产物54,956字节；此前54,624字节和54,752字节版本均不完整，不用于此次升级。详见[修复总结](../docs/STM32App启动修复与Keil烧录.md)。

`patches/health-startup.patch` 仅为此前健康初始化修复的历史记录，**不包含本次安全初始化修复**。仓库内App已包含其全部改动，不要再次应用。只应用这个旧补丁不足以得到本次完整修复，新的下载/开发应直接使用仓库内App。
