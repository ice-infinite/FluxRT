# 独立Python数值参考

此目录用于复现随包47张PNG、24份CSV及JSON指标。它不是MATLAB运行器，也不证明.m代码已经被执行。

本次环境：Python 3.13.5、NumPy 2.3.5、Matplotlib 3.10.8。程序未使用专用电机仿真库。

```bash
python reference_python/run_reference.py
```

从包根目录运行。程序重新写入`figures/`与`reference_results/`中的参考输出；修改代码前请备份基线。它不会生成`results_matlab`。

验证报告中`matlab_executed=false`、`simulink_executed=false`、`cross_language_numeric_equivalence_proven=false`为刻意保留的事实。先在本地运行MATLAB版本，再做同方程、同参数、同初值和同采样的逐样本比较，才可升级相应验证状态。
