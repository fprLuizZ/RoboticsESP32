#include <kinematics/kinematics.h>

Pose Kinematics::forward(JointAngles j)
{
    float c1 = cos(j.t1);
    float s1 = sin(j.t1);
    float c12 = cos(j.t1 + j.t2);
    float s12 = sin(j.t1 + j.t2);
    float c123 = cos(j.t1 + j.t2 + j.t3);
    float s123 = sin(j.t1 + j.t2 + j.t3);

    float r_planar = L1 * c1 + L2 * c12 + L3 * c123;

    Pose posePrevista;
    posePrevista.x = r_planar * cos(j.t0);
    posePrevista.y = r_planar * sin(j.t0);
    posePrevista.z = L1 * s1 + L2 * s12 + L3 * s123;
    posePrevista.phi = j.t1 + j.t2 + j.t3;

    return posePrevista;
}

Solution Kinematics::inverse(Pose p, JointAngles &j)
{
    const float PHI_STEP = 1.0;
    const float PHI_MAX_DELTA = 45;

    JointAngles calculados;
    Solution solution;
    solution.thetas_deg = j;

    for (float delta = 0; delta <= PHI_MAX_DELTA; delta += PHI_STEP)
    {

        for (int sinal = -1; sinal <= 1; sinal += 2)
        {
            // calculo TODO feito em radianos
            float phi_test = p.phi + sinal * delta;
            float phi = phi_test * DEG_TO_RAD;

            calculados.t0 = atan2(p.y, p.x);
            float r = sqrt(p.x * p.x + p.y * p.y);

            if (r < 1e-6)
                calculados.t0 = j.t0;

            float xw = r - L3 * cos(phi);
            float yw = p.z - L3 * sin(phi);
            float d = sqrt(xw * xw + yw * yw);

            if (d > L1 + L2 || d < fabs(L1 - L2))
                continue;

            float cos_theta2 =
                (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

            if (cos_theta2 > 1.0)
                cos_theta2 = 1.0;
            if (cos_theta2 < -1.0)
                cos_theta2 = -1.0;

            float theta2_pos = acos(cos_theta2);
            float theta2_neg = -theta2_pos;

            float psi = atan2(yw, xw);

            float sol[2][4];
            int n_sol = 0;

            for (int i = 0; i < 2; i++)
            {

                calculados.t2 = (i == 0) ? theta2_pos : theta2_neg;

                float alpha =
                    atan2(L2 * sin(calculados.t2),
                          L1 + L2 * cos(calculados.t2));

                calculados.t1 = psi - alpha;
                calculados.t3 = phi - calculados.t1 - calculados.t2;

                sol[n_sol][0] = calculados.t0;
                sol[n_sol][1] = calculados.t1;
                sol[n_sol][2] = calculados.t2;
                sol[n_sol][3] = calculados.t3;

                n_sol++;
            }

            int melhor = -1;
            float menor_custo = 1e9;

            for (int i = 0; i < n_sol; i++)
            {

                float s0 = thetaToServo(0, sol[i][0] * RAD_TO_DEG);
                float s1 = thetaToServo(1, sol[i][1] * RAD_TO_DEG);
                float s2 = thetaToServo(2, sol[i][2] * RAD_TO_DEG);
                float s3 = thetaToServo(3, sol[i][3] * RAD_TO_DEG);

                if (s0 < SERVO_MIN || s0 > SERVO_MAX &&
                    s1 < SERVO_MIN || s1 > SERVO_MAX &&
                    s2 < SERVO_MIN || s2 > SERVO_MAX &&
                    s3 < SERVO_MIN || s3 > SERVO_MAX)
                    continue;

                float custo =
                    fabs(sol[i][0] - j.t0) +
                    fabs(sol[i][1] - j.t1) +
                    fabs(sol[i][2] - j.t2) +
                    fabs(sol[i][3] - j.t3);

                if (custo < menor_custo)
                {
                    menor_custo = custo;
                    melhor = i;
                }
            }

            if (melhor != -1)
            {

                solution.exist = true;
                solution.thetas_deg.t0 = sol[melhor][0] * RAD_TO_DEG;
                solution.thetas_deg.t1 = sol[melhor][1] * RAD_TO_DEG;
                solution.thetas_deg.t2 = sol[melhor][2] * RAD_TO_DEG;
                solution.thetas_deg.t3 = sol[melhor][3] * RAD_TO_DEG;

                return solution;
            }

            if (delta == 0)
                break;
        }
    }

    solution.exist = false;
    Serial.println("Nenhuma solução encontrada.");
    return solution;
}

float Kinematics::thetaToServo(int joint, float theta_deg)
{
    return OFFSETS[joint] + DIRECTIONS[joint] * theta_deg;
}

/*
bool Kinematics::UpdateServos(JointAngles thetas_deg, JointAngles &j)
{
    JointAngles servoAngles;

    servoAngles.t0 = thetaToServo(0, thetas_deg.t0);
    servoAngles.t1 = thetaToServo(1, thetas_deg.t1);
    servoAngles.t2 = thetaToServo(2, thetas_deg.t2);
    servoAngles.t3 = thetaToServo(3, thetas_deg.t3);

    if (
        servoAngles.t0 < SERVO_MIN || servoAngles.t0 > SERVO_MAX ||
        servoAngles.t1 < SERVO_MIN || servoAngles.t1 > SERVO_MAX ||
        servoAngles.t2 < SERVO_MIN || servoAngles.t2 > SERVO_MAX ||
        servoAngles.t3 < SERVO_MIN || servoAngles.t3 > SERVO_MAX 
    
    )
    {
        Serial.println("Erro: ângulo resulta em servo fora dos limites");
        return false;
    }
    

    _servos[i].write(servoAngles[i]);

    j.t0 = thetas_deg.t0 * DEG_TO_RAD; // armazena em rad
    j.t1 = thetas_deg.t1 * DEG_TO_RAD; // armazena em rad
    j.t2 = thetas_deg.t2 * DEG_TO_RAD; // armazena em rad
    j.t3 = thetas_deg.t3 * DEG_TO_RAD; // armazena em rad

    return true;
}
*/