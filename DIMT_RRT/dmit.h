//
// Created by Olga Mikheeva on 28/01/16.
//

#include <vector>
#include <tuple>

std::pair<double,std::pair<double,double>> findMinTimeOneDOF(double p1, double p2,
                                                             double v1, double v2,
                                                             double v_max, double a_max) {
    //determine the sign (9) - (12)
    double delta_p_acc_x = 0.5 * (v1 + v2) * fabs(v2 - v1) / a_max;
    int sign = (p2 - p1 - delta_p_acc_x < 0)? -1 : 1;
    double a1 = sign * a_max;
    double a2 = -a1;
    double v_lim = sign * v_max;
    double T;
    std::pair<double, double> T_inf(0, 0);

    //(13)
    double a = a1;
    double b = 2 * v1;
    double c = (pow(v2, 2) - pow(v1, 2))/(2 * a2) - (p2 - p1);
    double q, t1, t2, v_t_a1;
    double t_a1, t_v, t_a2; //duration of time segments

    int sign_a = (a < 0)? -1 : 1;
    int sign_b = (b < 0)? -1 : 1;


    q = -0.5 * (b + sign_b * sqrt(pow(b, 2) - 4 * a * c));

    t1 = q / a;
    t2 = c / q;

    t_a1 = (sign_a == sign_b) ? t2 : t1;
    t_a2 = (v2 - v1) / a2 + t_a1;
    t_v = 0;

    //check for velocity limits at t_a1
    v_t_a1 = v1 + a1 * t_a1;

    if (fabs(v_t_a1) > v_max) {
        //(15)-(17)
        t_a1 = (v_lim - v1) / a1;
        t_a2 = (v2 - v_lim) / a2;
        t_v = (pow(v1, 2) + pow(v2, 2) - 2 * pow(v_lim, 2)) / (2 * v_lim * a1) + (p2 - p1) / v_lim;
    }

    T = t_a1 + t_a2 + t_v;



    //compute limits of infeasible time interval
    a1 = -sign * a_max;
    a2 = -a1;
    v_lim = -sign * v_max;
    a = a1;
    b = 2 * v1;
    c = (pow(v2, 2) - pow(v1, 2))/(2 * a2) - (p2 - p1);

    sign_a = (a < 0)? -1 : 1;
    sign_b = (b < 0)? -1 : 1;

    q = -0.5 * (b + sign_b * sqrt(pow(b, 2) - 4 * a * c));

    if (!isnan(q)) {

        t1 = q / a;
        t2 = c / q;

        t_a1 = (sign_a == sign_b) ? t1 : t2; //min - lower bound of infeasible interval
        t_a2 = (v2 - v1) / a2 + t_a1;
        t_v = 0;
        T_inf.first = t_a1 + t_a2 + t_v;

        t_a1 = (sign_a == sign_b) ? t2 : t1; //max - upper bound of infeasible interval
        t_a2 = (v2 - v1) / a2 + t_a1;
        t_v = 0;

        //check for velocity limits at t_a1 for upper bound
        v_t_a1 = v1 + a1 * t_a1;

        if (fabs(v_t_a1) > v_max) {
            //(15)-(17)
            t_a1 = (v_lim - v1) / a1;
            t_a2 = (v2 - v_lim) / a2;
            t_v = (pow(v1, 2) + pow(v2, 2) - 2 * pow(v_lim, 2)) / (2 * v_lim * a1) + (p2 - p1) / v_lim;
        }

        T_inf.second = t_a1 + t_a2 + t_v;
    }

    return std::pair<double, std::pair<double,double>> (T, T_inf);
}

double findMinOverallTime(std::vector<std::pair<double, std::pair<double,double>>> DOFs) {
    double T_min = 0; //max of min times
    for (auto dof: DOFs) {
        T_min = (T_min < dof.first)? dof.first : T_min;
    }

    //check for infeasible intervals

    bool changed = 1;

    while (changed) {
        changed = 0;
        for (auto dof: DOFs) {
            if (T_min > dof.second.first && T_min < dof.second.second) {
                T_min = dof.second.second;
                changed = 1;
            }
        }
    }

    return T_min;
}

std::tuple<std::pair<double, double>,std::pair<double, double>,std::pair<double, double>> findFixedTimeTrajectory(
        double p1, double p2, double v1, double v2, double v_max, double a_max, double T) {
    double a = pow(T, 2);
    double b = 2 * T * (v1 + v2) - 4 * (p2 - p1);
    double c = - pow(v2 - v1, 2);

    int sign_a = (a < 0)? -1 : 1;
    int sign_b = (b < 0)? -1 : 1;

    double q = -0.5 * (b + sign_b * sqrt(pow(b, 2) - 4 * a * c));
    double a1_1 = q / a;
    double a1_2 = c / q;

    double a1 = (fabs(a1_1) > fabs(a1_2))? a1_1 : a1_2;
    double a2 = -a1;
    double t_a1 = 0.5 * ((v2 - v1)/a1 + T);
    double t_a2 = T - t_a1;
    double t_v = 0;

    //check for velocity limits at t_a1 for upper bound
    double v_t_a1 = v1 + a1 * t_a1;
    double v_lim = 0;

    if (fabs(v_t_a1) > v_max) {
        std::cout<<"\nvelocity violation";
        int sign_a1 = (a1 < 0)? -1 : 1;
        v_lim = -sign_a1 * v_max;
        a1 = (pow(v_lim - v1, 2) + pow(v_lim - v2, 2))/(2 * (v_lim * T - (p2 - p1)));
        a2 = -a1;
        //(15)-(17)
        t_a1 = (v_lim - v1) / a1;
        t_a2 = (v2 - v_lim) / a2;
        t_v = (pow(v1, 2) + pow(v2, 2) - 2 * pow(v_lim, 2)) / (2 * v_lim * a1) + (p2 - p1) / v_lim;
    }
    std::cout<<"\nT_new="<<(t_a1+t_a2+t_v);

    return std::make_tuple(std::pair<double, double>(t_a1, a1),
                           std::pair<double, double>(t_v, v_lim),
                           std::pair<double, double>(t_a2, a2));
}

double findMinTime(std::pair<double, double> p1, std::pair<double, double> p2,
                   std::pair<double, double> v1, std::pair<double, double> v2,
                   std::pair<double, double> v_max, std::pair<double, double> a_max) {

    std::pair<double,std::pair<double,double>> Tx = findMinTimeOneDOF(p1.first, p2.first, v1.first, v2.first,
                                                                      v_max.first, a_max.first);
    std::pair<double,std::pair<double,double>> Ty = findMinTimeOneDOF(p1.second, p2.second, v1.second, v2.second,
                                                                      v_max.second, a_max.second);

    //compute overall min time
    std::vector<std::pair<double, std::pair<double,double>>> DOFs;
    DOFs.push_back(Tx);
    DOFs.push_back(Ty);
    double T_min = findMinOverallTime(DOFs);

    std::cout<<"\nx T="<<Tx.first<<", T_inf=["<<Tx.second.first<<", "<<Tx.second.second<<"]";
    std::cout<<"\ny T="<<Ty.first<<", T_inf=["<<Ty.second.first<<", "<<Ty.second.second<<"]";
    std::cout<<"\ny T_min="<<T_min;

    std::tuple<std::pair<double, double>,std::pair<double, double>,std::pair<double, double>> Trx =
            findFixedTimeTrajectory(p1.first, p2.first, v1.first, v2.first, v_max.first, a_max.first, T_min);
    std::tuple<std::pair<double, double>,std::pair<double, double>,std::pair<double, double>> Try =
            findFixedTimeTrajectory(p1.second, p2.second, v1.second, v2.second, v_max.second, a_max.second, T_min);

    
    return -1;
}

