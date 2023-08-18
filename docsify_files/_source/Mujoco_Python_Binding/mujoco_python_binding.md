# Mujoco Python Bindings

> [Mujoco的python接口](https://mujoco.readthedocs.io/en/stable/python.html)
>
> 在python中使用mujoco



mujoco在被DeepMind收购并开源后，推出了原生的python接口，这意味我们可以愉快得使用python进行mujoco相关的工作了。

但据mujoco开发人员所说，如果你使用C/C++进行mujoco进行开发工作，你的体验会爽上天。因为python只是相当于是一个接口，实际底层运行的还是C/C++，并且python在多线程上有先天性的劣势。

（但笔者目前写C/C++的水平相比python菜得扣jio，python就是我大爹！）打开viewer，把模型拖入......



下面是一个使用python binding的实例代码：

```python
import time

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path('example_3.xml')
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
```

第六行的模型我用的上一章的[example_3.xml](../Models in Mujoco/example_3.xml)，把模型和上面的python脚本文件放在同一个文件夹里（或指定模型的路径）



运行python脚本。