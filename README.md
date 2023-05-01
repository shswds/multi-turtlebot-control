# 프로젝트 목적

# turtlebot3_nps-master

- 여러 대의 터틀봇의 정보를 하나의 맵위에서 받아오기 위한 패키지임
- 맵은 2D Lidar를 사용하여 G-mapping을 통해 구성하였음
- 만들어진 맵 위에서 2D Lidar를 통해 현재 터틀봇 위치 좌표를 받아옴

# control code
- 처음에 ros1 package에서 사용하는 Quaternion 좌표와 비례상수를 이용한 피드백 제어를 진행하였음
- Quaternion로 좌표를 이용한 횡방향 제어기 설계를 하고 수정하는데 어려움을 겪음
- Quaternion 좌표를 Euler angle 좌표 변환하여 횡방향 제어기 설계를 하였고, 부드럽고 안정적으로 경로추정을 잘하게됨
