# APBP(Artificial Potential Based Planner)
### [2025 2학기 로봇알고리즘 과제3]

### 개발 기간
> 2025.10.02 ~ 2025.10.09

### 개발 환경
> Unity 6000.2.1f1<br>
> Templates : Universal 3D<br>
> RTX4050 Laptop<br>

### 설명
+ 동기
  + 로봇알고리즘 수업 세 번째 과제
+ 기획
  + Local Minimum
    + 로봇 위치 $q$에 대해 목표 지점으로는 끌어당기는 인력과, 장애물로부터 밀어내는 반발력을 합쳐서 전체 포텐셜 $U(q)$를 만들고, 그 기울기를 따라 이동함.<br>
    + 목표로 끌어당기는 인력 <br> ![equation](https://latex.codecogs.com/png.image?%5Cinline%20%5Cdpi%7B110%7D%5Cbg%7Bwhite%7DF_%7Batt%7D=%5Cbegin%7Bcases*%7D-%5Cfrac%7B%5Cepsilon%5Czeta%5Ctext%7B(current%20position%5C,-%5C,goal%20position)%7D%7D%7B%5Ctext%7Bdistance%20of%20goal%20position%7D%7D&%5Ctext%7Bif(distance%20of%20goal%20position%7D%5C;%3E%5Cepsilon%5Ctext%7B)%7D%5C%5C-%5Czeta%5Ctext%7B(current%20position%5C,-%5C,goal%20position)%7D&%5Ctext%7Bif(distance%20of%20goal%20position%7D%5C;%5Cleq%5Cepsilon%5Ctext%7B)%7D%5Cend%7Bcases*%7D) <br>
    + 장애물로부터 밀어내는 반발력 <br> ![equation](https://latex.codecogs.com/png.image?%5Cinline%20%5Cdpi%7B110%7D%5Cbg%7Bwhite%7DF_%7Brep%7D=%5Cbegin%7Bcases*%7D%5Cfrac%7B%5Ceta%7D%7B2%7D(%5Cfrac%7B1%7D%7B%5Ctext%7Bdistance%20of%20obstacle%7D%7D%5C,-%5C,%5Cfrac%7B1%7D%7B%5Cdelta%7D)%5Cfrac%7B1%7D%7B%5Ctext%7Bdistance%20of%20obstacle%7D%5E2%7D%5Cfrac%7B(%5Ctext%7Bcurrent%20position%5C,-%5C,obstacle%20position%7D)%7D%7B%5Ctext%7Bdistance%20of%20obstacle%7D%7D&%5Ctext%7Bif%5C;(distance%20of%20obstacle%7D%3C%5Cdelta%5Ctext%7B)%7D%5C%5C0&%5Ctext%7Bif%5C;(distance%20of%20obstacle%7D%5Cgeq%5Cdelta%5Ctext%7B)%7D%5Cend%7Bcases*%7D)
  + 무인 자동차
    + Jacobian을 이용하여 work space의 힘을 configuration space로 옮겨서 자동차의 위치를 이동함.
    + Configuration Space : $q = [x, y, \theta]$
    + $p_1 = (0, 0), \\ p_2 = (1.5, 0), \\ p_3 = (1.5, -1)$
    + $\Rightarrow p_1 = (x, y), \\ p_2 = (x + 1.5 cos(\theta), y + 1.5 sin(\theta)), \\ p_3 = (x + 1.5 cos(\theta) - sin(\theta), y + 1.5 sin(\theta) + cos(\theta))$
    + $J_1 = \frac{\partial p_1}{\partial q}, \\ J_2 = \frac{\partial p_2}{\partial q}, \\ J_3 = \frac{\partial p_3}{\partial q}$
    + 각 point에 맞는 인력의 총 합 $F_Q(q) = \sum_{i=1}^{3} J^T_i \cdot F_{att}$를 계산
    + configuration space에서 $F_Q(q)$만큼 이동 $q' \leftarrow q + F_Q(q)$
  + UAV
    + Configuration Space : $q = [x, y, z, \theta_x, \theta_y, \theta_z]$
    + $p_1 = (-0.67, 0, -1.38), \\ p_2 = (-0.67, 0, 1.38), \\ p_3 = (0.71, 0, 0)$
    + $H = R_{roll} \cdot R_{pitch} \cdot R_{yaw}$
    + ![equation](https://latex.codecogs.com/png.image?%5Cinline%20%5Cdpi%7B110%7D%5Cbg%7Bwhite%7D=%5Cbegin%7Bbmatrix%7D1&0&0&x%5C%5C0&cos(%5Ctheta_%7Bx%7D)&-sin(%5Ctheta_%7Bx%7D)&y%5C%5C0&sin(%5Ctheta_%7Bx%7D)&cos(%5Ctheta_%7Bx%7D)&z%5C%5C0&0&0&1%5C%5C%5Cend%7Bbmatrix%7D%5Ccdot%5Cbegin%7Bbmatrix%7Dcos(%5Ctheta_%7By%7D)&0&-sin(%5Ctheta_%7By%7D)&x%5C%5C0&1&0&y%5C%5Csin(%5Ctheta_%7By%7D)&0&cos(%5Ctheta_%7By%7D)&z%5C%5C0&0&0&1%5C%5C%5Cend%7Bbmatrix%7D%5Ccdot%5Cbegin%7Bbmatrix%7Dcos(%5Ctheta_%7Bz%7D)&-sin(%5Ctheta_%7Bz%7D)&0&x%5C%5Csin(%5Ctheta_%7Bz%7D)&cos(%5Ctheta_%7Bz%7D)&0&y%5C%5C0&0&1&z%5C%5C0&0&0&1%5C%5C%5Cend%7Bbmatrix%7D%5Ccdot%20)
    + ![equation](https://latex.codecogs.com/png.image?%5Cinline%20%5Cdpi%7B110%7D%5Cbg%7Bwhite%7Dp_%7B1%7D=H%5Ccdot%5Cbegin%7Bbmatrix%7D-0.67%5C%5C0%5C%5C-1.38%5C%5C1%5Cend%7Bbmatrix%7D,%5C;p_%7B2%7D=H%5Ccdot%5Cbegin%7Bbmatrix%7D-0.67%5C%5C0%5C%5C1.38%5C%5C1%5Cend%7Bbmatrix%7D,%5C;p_%7B3%7D=H%5Ccdot%5Cbegin%7Bbmatrix%7D0.71%5C%5C0%5C%5C0%5C%5C1%5Cend%7Bbmatrix%7D)
    + $J_{i} = \frac{\partial p_i}{\partial q}$ <br><br>

### Local Minimum 실행결과

https://github.com/user-attachments/assets/70415d95-23a4-4801-834e-3d479116e822

https://github.com/user-attachments/assets/7035b480-12af-4e0b-b170-9edb3af49f68

### 무인 자동차 실행결과

https://github.com/user-attachments/assets/30c75391-f95e-4bf9-8bdd-de1ce762e721

### UAV 실행결과

https://github.com/user-attachments/assets/13181d93-cdc4-4cdc-beaf-991c9f521158

<br>
