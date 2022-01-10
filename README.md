This is a simple wrapper over pybulle simulation. This is an
oversimplification of RoboVat, and it is designed for fast development
of research topics in Task and Motion Planning as well as fast testing
of various robot control tasks.


## Download assets

``` shell
cd bullet_world
wget https://utexas.box.com/shared/static/c4xu0u5udnkd4ho68t5py34v29yq7tgv.zip -O assets.zip
unzip assets.zip
```

## Run example
1. Visualize pybullet simulation in NViSII (You need to install
[NViSII](https://github.com/owl-project/NVISII), currently tested
under version `1.0.70` with cuda 11.0, nvidia-driver `450.51.05`):
``` shell
	python scripts/test_bullet_world.py
```

1. Run simple ik example:

``` shell
	python scripts/test_position_control.py
```
