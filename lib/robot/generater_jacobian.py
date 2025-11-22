# --------------------------------------
# 자코비안 행렬 계산
#---------------------------------------
import sympy as sp
from sympy import Matrix, cos, sin, atan2, acos, sqrt, diff

def Trans(x, y, z):
    """
    4x4 동차 평행이동 행렬 생성
    """
    return Matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def Rot(angle, axis):
    """
    4x4 동차 회전 행렬 생성 (Angle-Axis)
    """
    kx, ky, kz = axis
    # 축 벡터가 이미 정규화되었다고 가정하거나, 필요시 정규화
    norm = sqrt(kx**2 + ky**2 + kz**2)
    kx /= norm; ky /= norm; kz /= norm;

    c = cos(angle)
    s = sin(angle)
    v = 1 - c

    return Matrix([
        [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s, 0],
        [kx*ky*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s, 0],
        [kx*kz*v - ky*s, ky*kz*v + kx*s, kz*kz*v + c,    0],
        [0,              0,              0,              1]
    ])

def compute_jacobian(fk):
    """
    FK 행렬로부터 위치 및 회전 행렬 요소에 대한 자코비안 계산
    """

    x = fk[0, 3]
    y = fk[1, 3]
    z = fk[2, 3]



    wx = atan2(fk[1, 2], fk[0, 2])   # α
    wy = acos(fk[2, 2])              # β
    wz = atan2(fk[2, 1], -fk[2, 0])  # γ


    jacoban = sp.zeros(6, 6)

    jacoban[0, 0] = diff(x, q0);
    jacoban[0, 1] = diff(x, q1);
    jacoban[0, 2] = diff(x, q2);
    jacoban[0, 3] = diff(x, q3);
    jacoban[0, 4] = diff(x, q4);
    jacoban[0, 5] = diff(x, q5);

    jacoban[1, 0] = diff(y, q0);
    jacoban[1, 1] = diff(y, q1);
    jacoban[1, 2] = diff(y, q2);
    jacoban[1, 3] = diff(y, q3);
    jacoban[1, 4] = diff(y, q4);
    jacoban[1, 5] = diff(y, q5);

    jacoban[2, 0] = diff(z, q0);
    jacoban[2, 1] = diff(z, q1);
    jacoban[2, 2] = diff(z, q2);
    jacoban[2, 3] = diff(z, q3);
    jacoban[2, 4] = diff(z, q4);
    jacoban[2, 5] = diff(z, q5);

    jacoban[3, 0] = diff(wx, q0);
    jacoban[3, 1] = diff(wx, q1);
    jacoban[3, 2] = diff(wx, q2);
    jacoban[3, 3] = diff(wx, q3);
    jacoban[3, 4] = diff(wx, q4);
    jacoban[3, 5] = diff(wx, q5);

    jacoban[4, 0] = diff(wy, q0);
    jacoban[4, 1] = diff(wy, q1);
    jacoban[4, 2] = diff(wy, q2);
    jacoban[4, 3] = diff(wy, q3);
    jacoban[4, 4] = diff(wy, q4);
    jacoban[4, 5] = diff(wy, q5);

    jacoban[5, 0] = diff(wz, q0);
    jacoban[5, 1] = diff(wz, q1);
    jacoban[5, 2] = diff(wz, q2);
    jacoban[5, 3] = diff(wz, q3);
    jacoban[5, 4] = diff(wz, q4);
    jacoban[5, 5] = diff(wz, q5);

    return jacoban




if __name__ == "__main__":
# --------------------------------------
# | 로봇 FK 및 자코비안 계산을 위한 툴
# | FK는 회전 -> 이동 순으로 계산해야함
# --------------------------------------
    print("자코비안 계산 프로그램 실행...")



# --------------------------------------
# 심볼릭 변수 정의
# --------------------------------------
    q0, q1, q2, q3, q4, q5 = sp.symbols('q0 q1 q2 q3 q4 q5', real=True)
    l1, l2, l3, l4, l5, l6, l7, l8 = sp.symbols('l1 l2 l3 l4 l5 l6 l7 l8', positive=True, real=True)



# --------------------------------------
# | FK 계산
# | 회전 축을 Z축으로 하는 걸 고집했는데.. 그렇게 하지 않아도 잘됨
# | 회전 먼저 계산 후 이동 계산을 수행해야함 ( Transfrom 클래스 구조가 이렇게 되어 있음)
# | tip: Jacobian이 잘 안풀리는 경우에는 끝단의 방향만 반대로 바꺼주면 되는 경우도 있다고 함
# --------------------------------------
    base_tf = Trans(0, 0, 0)                                     # Base
    j1_tf   = Trans(0, 0, l1)      * Rot(q0, [0, 0, 1])           # Joint1
    j2_tf   = Trans(-l2, 0, 0)     * Rot(q1, [1, 0, 0])           # Joint2
    j3_tf   = Trans(0, 0, l3)      * Rot(q2, [1, 0, 0])           # Joint3
    j4_tf   = Trans(l4, 0, 0)      * Rot(q3, [0, 0, 1])           # Joint4
    j5_tf   = Trans(-l6, 0, l5)    * Rot(q4, [1, 0, 0])           # Joint5
    j6_tf   = Trans(l7, 0, 0)      * Rot(q5, [0, 0, 1])           # Joint6
    end_effector_tf = Trans(0, 0, l8) * Rot(0, [0, 0, 1])          # End Effector


    FK = base_tf * j1_tf * j2_tf * j3_tf * j4_tf * j5_tf * j6_tf * end_effector_tf



# --------------------------------------
# 자코비안 계산
# --------------------------------------
    print("\n자코비안 계산 중...")
    joint_vars = [q0, q1, q2, q3, q4, q5]
    J = compute_jacobian(FK)
    # J = sp.simplify(J)



# --------------------------------------
# 자코비안 행렬 cpp 스타일로 출력
# --------------------------------------
    print("\n--- 생성된 자코비안 (C++ 형식) ---")
    for i in range(J.rows):
        for j in range(J.cols):
            print(f"J({i},{j}) = {sp.cxxcode(J[i, j])};")
