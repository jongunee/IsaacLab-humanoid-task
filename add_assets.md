### rby1 모델 변환
```
./isaaclab.sh -p source/standalone/tools/convert_urdf.py \
  ./robot_assets/rby1a/urdf/model.urdf \
  source/extensions/omni.isaac.lab_assets/data/Robots/rainbowrobotics/rby1.usd
```

### Ur3e + 2f-88 모델 변환
```
./isaaclab.sh -p source/standalone/tools/convert_urdf.py \
  ./robot_assets/ur3e_description/robots/ur3e_2f85.urdf \
  source/extensions/omni.isaac.lab_assets/data/Robots/Universal_Robots/ur3e_2f85.usd