import sympy as sp
from sympy import Matrix, cos, sin, sqrt, diff, atan2, acos

def Trans(x, y, z):
    """
    4x4 동차 평행이동 행렬
    """
    return Matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])


def Rot(angle, axis):
    """
    4x4 동차 회전 행렬 (Angle-Axis)
    """
    kx, ky, kz = axis
    # 축 벡터 정규화 가정 (단위 벡터여야 함)
    c = cos(angle)
    s = sin(angle)
    v = 1 - c

    return Matrix([
        [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s, 0],
        [kx*ky*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s, 0],
        [kx*kz*v - ky*s, ky*kz*v + kx*s, kz*kz*v + c,    0],
        [0,              0,              0,              1]
    ])


def compute_jacobian_matrix_derivative(fk, joint_vars):
    """
    회전 행렬의 미분(dR/dq)과 전치(R^T)를 이용한 자코비안 계산
    관계식: [w]x = (dR / dq) * R^T
    """
    
    # 1. 위치 벡터 (Position)
    pos = fk[0:3, 3]


    # 2. 회전 행렬 (Rotation Matrix)
    R = fk[0:3, 0:3]


    # 자코비안 행렬 초기화 (6 rows x N joints)
    n_joints = len(joint_vars)
    J = sp.zeros(6, n_joints)



    for i, q in enumerate(joint_vars):
        # (1) 선속도 (Linear Velocity, J_v)
        # 위치 벡터를 관절 변수 q로 편미분
        J[0, i] = diff(pos[0], q)
        J[1, i] = diff(pos[1], q)
        J[2, i] = diff(pos[2], q)


        # (2) 각속도 (Angular Velocity, J_w)
        # 회전 행렬 미분 공식 이용: S(w) = dR/dq * R^T
        dR_dq = diff(R, q)
        S = dR_dq * R.T


        # S는 Skew-Symmetric Matrix (반대칭 행렬) 형태임
        # [  0, -wz,  wy ]
        # [ wz,   0, -wx ]
        # [-wy,  wx,   0 ]

        # 행렬 요소에서 각속도 성분 추출
        J[3, i] = S[2, 1]  # wx
        J[4, i] = S[0, 2]  # wy
        J[5, i] = S[1, 0]  # wz

    return J # 6xN


def compute_jacobian_matrix(fk, joint_vars):
    """
    FK 행렬로부터 위치 및 회전 행렬 요소에 대한 자코비안 계산
    """

    elements = Matrix([
        fk[0, 3], fk[1, 3], fk[2, 3],  # x, y, z
        fk[0, 0], fk[0, 1], fk[0, 2],  # r11, r12, r13
        fk[1, 0], fk[1, 1], fk[1, 2],  # r21, r22, r23
        fk[2, 0], fk[2, 1], fk[2, 2]   # r31, r32, r33
    ])

    # 모든 요소에 대해 각 조인트 변수로 편미분하여 자코비안 계산
    J = elements.jacobian(joint_vars)

    return J # 12xN


def compute_jacobian_rpy(fk, joint_vars):
    """
    오일러 회전(ZYZ) 행렬을 이용한 자코비안 계산
    """

    x = fk[0, 3]
    y = fk[1, 3]
    z = fk[2, 3]

    wx = atan2(fk[1, 2], fk[0, 2])  # α
    wy = acos(fk[2, 2])             # β
    wz = atan2(fk[2, 1], -fk[2, 0]) # γ


    jacoban = sp.zeros(6, 6)

    for j in range(len(joint_vars)):
        jacoban[0, j] = diff(x, joint_vars[j])
        jacoban[1, j] = diff(y, joint_vars[j])
        jacoban[2, j] = diff(z, joint_vars[j])
        jacoban[3, j] = diff(wx, joint_vars[j])
        jacoban[4, j] = diff(wy, joint_vars[j])
        jacoban[5, j] = diff(wz, joint_vars[j])

    return jacoban # 6x6






if __name__ == "__main__":
    print("로봇 자코비안 생성기 실행..")
# -----------------------------------------------
# 1 | 심볼 정의 (로봇 링크 길이 및 관절 변수)
# -----------------------------------------------
    q0, q1, q2, q3, q4, q5 = sp.symbols('q0 q1 q2 q3 q4 q5', real=True)
    l1, l2, l3, l4, l5, l6, l7, l8 = sp.symbols('l1 l2 l3 l4 l5 l6 l7 l8', positive=True, real=True)
    
    joint_vars = [q0, q1, q2, q3, q4, q5]


# -----------------------------------------------
# 2 | FK 구성 ( 두산 M1013 )
# -----------------------------------------------
    base_tf = Trans(0, 0, 0)                             # Base
    j1_tf   = Trans(0, 0, l1)      * Rot(q0, [0, 0, 1])  # Joint 1 (Z-axis)
    j2_tf   = Trans(-l2, 0, 0)     * Rot(q1, [1, 0, 0])  # Joint 2 (X-axis)
    j3_tf   = Trans(0, 0, l3)      * Rot(q2, [1, 0, 0])  # Joint 3 (X-axis)
    j4_tf   = Trans(l4, 0, 0)      * Rot(q3, [0, 0, 1])  # Joint 4 (Z-axis)
    j5_tf   = Trans(-l6, 0, l5)    * Rot(q4, [1, 0, 0])  # Joint 5 (X-axis)
    j6_tf   = Trans(l7, 0, 0)      * Rot(q5, [0, 0, 1])  # Joint 6 (Z-axis)
    ee_tf   = Trans(0, 0, l8)      * Rot(0, [0, 0, 1])   # End Effector


    # 전체 FK 행렬 계산
    FK = base_tf * j1_tf * j2_tf * j3_tf * j4_tf * j5_tf * j6_tf * ee_tf

    # 자코비안 계산
    J = compute_jacobian_matrix_derivative(FK, joint_vars)
    print("Simplify..")
    J.simplify()


# -----------------------------------------------
# 3 | C++ 코드 출력
# -----------------------------------------------
    print("\n\n")
    print("mat<6, 6> Jcobian(double q0, double q1, double q2, double q3, double q4, double q5)")
    print("{")
    print("    mat<6, 6> J = mat<6, 6>::Zero();")
    print("")

    for row in range(6):
        for col in range(6):
            val = J[row, col]
            # 0인 경우 출력 생략하여 코드 길이 단축
            if val == 0:
                continue

            # C++ 문법으로 변환 (sin -> std::sin 등)
            ccode = sp.cxxcode(val)
            print(f"    J({row},{col}) = {ccode};")

    print("")
    print("    return J;")
    print("}")
    print("")
    print("")
