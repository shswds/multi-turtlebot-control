# 프로젝트 목적

# turtlebot3_nps-master

- 여러 대의 터틀봇의 정보를 하나의 맵위에서 받아오기 위한 패키지임
- 맵은 2D Lidar를 사용하여 G-mapping을 통해 구성하였음
- 만들어진 맵 위에서 2D Lidar를 통해 현재 터틀봇 위치 좌표를 받아옴

# control code
- 처음에 Euler angle과 비례상수를 이용한 피드백 제어를 진행하였음
- Euler angle로 횡방향 제어를 함에 있어 경로추종을 잘 못하고 부드럽게 제어가 잘 안되서 Quaternion 좌표게를 이용하여 해결함
