## URDF to USD
- `--merge-joints`: mimic 같은 중복된 조인트들을 간결화
- `--make-instanceable`: USD 파일을 복제하는 것이 아니라 참조 형태로 사용할 수 있도록 함
- 오류 나는건 확인 필요할듯

### rby1 모델 변환
```
./isaaclab.sh -p source/standalone/tools/convert_urdf.py \
  ./robot_assets/rby1a/urdf/model.urdf \
  source/extensions/omni.isaac.lab_assets/data/Robots/rainbowrobotics/rby1.usd\
  --merge-joints \
  --make-instanceable
```

### Ur3e + 2f-88 모델 변환
```
./isaaclab.sh -p source/standalone/tools/convert_urdf.py \
  ./robot_assets/ur3e_description/robots/ur3e_2f85.urdf \
  source/extensions/omni.isaac.lab_assets/data/Robots/Universal_Robots/ur3e_2f85.usd \
  --merge-joints \
  --make-instanceable
```