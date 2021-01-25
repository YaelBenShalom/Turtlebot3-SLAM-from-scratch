#include "../include/rigid2d/rigid2d.hpp"

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>

using namespace rigid2d;

int main(){
    char frame;
    Vector2D v, v_a, v_b, v_c;
    Twist2D twist, twist_a, twist_b, twist_c;
    Transform2D Tab, Tbc, Tba, Tcb, Tac, Tca;

    // Prompt the user to enter transforms Tab and Tbc:
    std::cout << "Enter transform Tab: " << std::endl;
    std::cin >> Tab;
	std::cout << "Enter transform Tbc: " << std::endl;
	std::cin >> Tbc;

    // Compute and output the transforms Tab, Tba, Tbc, Tcb, Tac, Tca:
    std::cout << "Tab = " << Tab << std::endl;
    Tba = Tab.inv();
    std::cout << "Tba = " << Tba << std::endl;
    std::cout << "Tbc = " << Tbc << std::endl;
    Tcb = Tbc.inv();
    std::cout << "Tcb = " << Tcb << std::endl;
    Tac = Tab * Tbc;
    std::cout << "Tac = " << Tac << std::endl;
    Tca = Tac.inv();
    std::cout << "Tca = " << Tca << std::endl;

    // Prompt the user to enter a vector and the frame in which it is defined:
    std::cout << "Enter Vector: " << std::endl;
    std::cin >> v;
	std::cout << "Enter Twist: " << std::endl;
	std::cin >> twist;
	std::cout << "Enter the frame in which the Vector and Twist are defined (a, b, or c):" << std::endl;
	std::cin >> frame;

    // Compute and output the vector in frames a, b, and c:
	if (frame == 'a'){
        std::cout << "The Vector in frame a: " << v << std::endl;
		v_b = Tba(v);
        std::cout << "The Vector in frame b: " << v_b << std::endl;
        v_c = Tca(v);
        std::cout << "The Vector in frame c: " << v_c << std::endl;

        std::cout << "The Twist in frame a: " << twist << std::endl;
		twist_b = Tba(twist);
        std::cout << "The Twist in frame b: " << twist_b << std::endl;
        twist_c = Tca(twist);
        std::cout << "The Twist in frame c: " << twist_c << std::endl;
	}

    else if (frame == 'b'){
		v_a = Tab(v);
        std::cout << "The Vector in frame a: " << v_a << std::endl;
        std::cout << "The Vector in frame b: " << v << std::endl;
        v_c = Tcb(v);
        std::cout << "The Vector in frame c: " << v_c << std::endl;

        twist_a = Tab(twist);
        std::cout << "The Twist in frame a: " << twist_a << std::endl;
        std::cout << "The Twist in frame b: " << twist << std::endl;
        twist_c = Tcb(twist);
        std::cout << "The Twist in frame c: " << twist_c << std::endl;
    }
    
    else {
		v_a = Tac(v);
        std::cout << "The Vector in frame a: " << v_a << std::endl;
        v_b = Tbc(v);
        std::cout << "The Vector in frame b: " << v_b << std::endl;
        std::cout << "The Vector in frame c: " << v << std::endl;

        twist_a = Tac(twist);
        std::cout << "The Twist in frame a: " << twist_a << std::endl;
        twist_b = Tbc(twist);
        std::cout << "The Twist in frame b: " << twist_b << std::endl;
        std::cout << "The Twist in frame c: " << twist << std::endl;
	}

    return 0;
}


