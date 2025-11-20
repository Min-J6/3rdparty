# --------------------------------------
# 자코비안 행렬 계산
#---------------------------------------
import sympy as sp
from sympy import Matrix, cos, sin

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

def compute_jacobian(fk, joint_vars):
    """
    FK 행렬로부터 위치 및 회전 행렬 요소에 대한 자코비안 계산
    """
    elements = Matrix([
        fk[0, 3], fk[1, 3], fk[2, 3],  # Position
        fk[0, 0], fk[0, 1], fk[0, 2],  # Rotation Matrix Row 1
        fk[1, 0], fk[1, 1], fk[1, 2],  # Rotation Matrix Row 2
        fk[2, 0], fk[2, 1], fk[2, 2]   # Rotation Matrix Row 3
    ])
    
    # 모든 요소에 대해 각 조인트 변수로 편미분하여 자코비안 계산
    jacobian = elements.jacobian(joint_vars)
    return jacobian




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
    base_tf = Trans(0, 0, 0)                            # Base
    j1_tf   = Rot(q0, [0, 0, 1]) * Trans(0, 0, l1)      # Joint1
    j2_tf   = Rot(q1, [1, 0, 0]) * Trans(-l2, 0, 0)     # Joint2
    j3_tf   = Rot(q2, [1, 0, 0]) * Trans(0, 0, l3)      # Joint3
    j4_tf   = Rot(q3, [0, 0, 1]) * Trans(l4, 0, 0)      # Joint4
    j5_tf   = Rot(q4, [1, 0, 0]) * Trans(-l6, 0, l5)    # Joint5
    j6_tf   = Rot(q5, [0, 0, 1]) * Trans(l7, 0, 0)      # Joint6
    end_effector_tf = Trans(0, 0, l8) * Rot(0, [0, 0, 1]) # End Effector ( TODO: 필요한 경우 사용하지만, offset 기능을 추가할까 함 )


    FK = base_tf * j1_tf * j2_tf * j3_tf * j4_tf * j5_tf * j6_tf * end_effector_tf



# --------------------------------------
# 자코비안 계산
# --------------------------------------
    print("\n자코비안 계산 중...")
    joint_vars = [q0, q1, q2, q3, q4, q5]
    J = compute_jacobian(FK, joint_vars)
    J = sp.simplify(J)



# --------------------------------------
# 자코비안 행렬 cpp 스타일로 출력
# --------------------------------------
    print("\n--- 생성된 자코비안 (C++ 형식) ---")
    for i in range(J.rows):
        for j in range(J.cols):
            print(f"J({i},{j}) = {sp.cxxcode(J[i, j])};")
