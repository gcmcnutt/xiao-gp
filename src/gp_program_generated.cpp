// Auto-generated GP evaluator function
//
// Source GP Information:
//   S3 Key: autoc-9223370282321121134-2025-08-07T02:27:34.673Z/gen9900.dmp
//   Generation: 9900
//   Original Length: 99
//   Original Depth: 20
//   Fitness: 78.1334
//   Bytecode Instructions: 99
//
#include "gp_program.h"

double generatedGPProgram(PathProvider& pathProvider, AircraftState& aircraftState, double arg) {
    double stack[16];
    int sp = 0;

    stack[sp++] = evaluateGPOperator(34, pathProvider, aircraftState, nullptr, 0); // ZERO
    stack[sp++] = evaluateGPOperator(22, pathProvider, aircraftState, nullptr, 0); // GETVELX
    // ATAN2
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(28, pathProvider, aircraftState, args, 2);
    }
    // SETTHROTTLE
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(19, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(34, pathProvider, aircraftState, nullptr, 0); // ZERO
    // GETDPHI
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(9, pathProvider, aircraftState, args, 1);
    }
    // SETPITCH
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(17, pathProvider, aircraftState, args, 1);
    }
    // EQ
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(5, pathProvider, aircraftState, args, 2);
    }
    // COS
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(8, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(13, pathProvider, aircraftState, nullptr, 0); // GETVEL
    stack[sp++] = evaluateGPOperator(34, pathProvider, aircraftState, nullptr, 0); // ZERO
    // EQ
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(5, pathProvider, aircraftState, args, 2);
    }
    // MAX
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(32, pathProvider, aircraftState, args, 2);
    }
    // GETDPHI
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(9, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(22, pathProvider, aircraftState, nullptr, 0); // GETVELX
    stack[sp++] = evaluateGPOperator(15, pathProvider, aircraftState, nullptr, 0); // GETROLL
    // GT
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(6, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(12, pathProvider, aircraftState, nullptr, 0); // GETDHOME
    // SQRT
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(30, pathProvider, aircraftState, args, 1);
    }
    // DIV
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(3, pathProvider, aircraftState, args, 2);
    }
    // SETROLL
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(18, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(24, pathProvider, aircraftState, nullptr, 0); // GETVELZ
    stack[sp++] = evaluateGPOperator(22, pathProvider, aircraftState, nullptr, 0); // GETVELX
    // GT
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(6, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(13, pathProvider, aircraftState, nullptr, 0); // GETVEL
    stack[sp++] = evaluateGPOperator(15, pathProvider, aircraftState, nullptr, 0); // GETROLL
    // GT
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(6, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(12, pathProvider, aircraftState, nullptr, 0); // GETDHOME
    // SQRT
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(30, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(12, pathProvider, aircraftState, nullptr, 0); // GETDHOME
    // GETDTARGET
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(11, pathProvider, aircraftState, args, 1);
    }
    // MAX
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(32, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(15, pathProvider, aircraftState, nullptr, 0); // GETROLL
    // GETDPHI
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(9, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(22, pathProvider, aircraftState, nullptr, 0); // GETVELX
    // SIN
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(7, pathProvider, aircraftState, args, 1);
    }
    // SUB
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(1, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(12, pathProvider, aircraftState, nullptr, 0); // GETDHOME
    // SQRT
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(30, pathProvider, aircraftState, args, 1);
    }
    // DIV
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(3, pathProvider, aircraftState, args, 2);
    }
    // ATAN2
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(28, pathProvider, aircraftState, args, 2);
    }
    // CLAMP
    {
        double args[3] = {stack[sp-3], stack[sp-2], stack[sp-1]};
        sp -= 3;
        stack[sp++] = evaluateGPOperator(27, pathProvider, aircraftState, args, 3);
    }
    stack[sp++] = evaluateGPOperator(34, pathProvider, aircraftState, nullptr, 0); // ZERO
    stack[sp++] = evaluateGPOperator(14, pathProvider, aircraftState, nullptr, 0); // GETPITCH
    // SUB
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(1, pathProvider, aircraftState, args, 2);
    }
    // SETPITCH
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(17, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(21, pathProvider, aircraftState, nullptr, 0); // GETBETA
    // GETDTHETA
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(10, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(33, pathProvider, aircraftState, nullptr, 0); // OP_PI
    stack[sp++] = evaluateGPOperator(21, pathProvider, aircraftState, nullptr, 0); // GETBETA
    stack[sp++] = evaluateGPOperator(15, pathProvider, aircraftState, nullptr, 0); // GETROLL
    // CLAMP
    {
        double args[3] = {stack[sp-3], stack[sp-2], stack[sp-1]};
        sp -= 3;
        stack[sp++] = evaluateGPOperator(27, pathProvider, aircraftState, args, 3);
    }
    // SETPITCH
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(17, pathProvider, aircraftState, args, 1);
    }
    // ADD
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(0, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(12, pathProvider, aircraftState, nullptr, 0); // GETDHOME
    // SQRT
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(30, pathProvider, aircraftState, args, 1);
    }
    // DIV
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(3, pathProvider, aircraftState, args, 2);
    }
    // SETROLL
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(18, pathProvider, aircraftState, args, 1);
    }
    // SETPITCH
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(17, pathProvider, aircraftState, args, 1);
    }
    // EQ
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(5, pathProvider, aircraftState, args, 2);
    }
    // COS
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(8, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(34, pathProvider, aircraftState, nullptr, 0); // ZERO
    // GETDPHI
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(9, pathProvider, aircraftState, args, 1);
    }
    // SETTHROTTLE
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(19, pathProvider, aircraftState, args, 1);
    }
    // MAX
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(32, pathProvider, aircraftState, args, 2);
    }
    // GETDTARGET
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(11, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(24, pathProvider, aircraftState, nullptr, 0); // GETVELZ
    stack[sp++] = evaluateGPOperator(22, pathProvider, aircraftState, nullptr, 0); // GETVELX
    // GT
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(6, pathProvider, aircraftState, args, 2);
    }
    // PROGN
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(37, pathProvider, aircraftState, args, 2);
    }
    // SIN
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(7, pathProvider, aircraftState, args, 1);
    }
    // ADD
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(0, pathProvider, aircraftState, args, 2);
    }
    // SIN
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(7, pathProvider, aircraftState, args, 1);
    }
    // ADD
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(0, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(23, pathProvider, aircraftState, nullptr, 0); // GETVELY
    // GETDTARGET
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(11, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(36, pathProvider, aircraftState, nullptr, 0); // TWO
    // GETDTHETA
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(10, pathProvider, aircraftState, args, 1);
    }
    // SUB
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(1, pathProvider, aircraftState, args, 2);
    }
    stack[sp++] = evaluateGPOperator(34, pathProvider, aircraftState, nullptr, 0); // ZERO
    // GETDPHI
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(9, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(23, pathProvider, aircraftState, nullptr, 0); // GETVELY
    stack[sp++] = evaluateGPOperator(36, pathProvider, aircraftState, nullptr, 0); // TWO
    // PROGN
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(37, pathProvider, aircraftState, args, 2);
    }
    // DIV
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(3, pathProvider, aircraftState, args, 2);
    }
    // SETROLL
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(18, pathProvider, aircraftState, args, 1);
    }
    stack[sp++] = evaluateGPOperator(35, pathProvider, aircraftState, nullptr, 0); // ONE
    stack[sp++] = evaluateGPOperator(26, pathProvider, aircraftState, nullptr, 0); // GETPITCH_RAD
    stack[sp++] = evaluateGPOperator(13, pathProvider, aircraftState, nullptr, 0); // GETVEL
    stack[sp++] = evaluateGPOperator(15, pathProvider, aircraftState, nullptr, 0); // GETROLL
    // GT
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(6, pathProvider, aircraftState, args, 2);
    }
    // CLAMP
    {
        double args[3] = {stack[sp-3], stack[sp-2], stack[sp-1]};
        sp -= 3;
        stack[sp++] = evaluateGPOperator(27, pathProvider, aircraftState, args, 3);
    }
    // MUL
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(2, pathProvider, aircraftState, args, 2);
    }
    // SETTHROTTLE
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(19, pathProvider, aircraftState, args, 1);
    }
    // MAX
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(32, pathProvider, aircraftState, args, 2);
    }
    // MAX
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(32, pathProvider, aircraftState, args, 2);
    }
    // PROGN
    {
        double args[2] = {stack[sp-2], stack[sp-1]};
        sp -= 2;
        stack[sp++] = evaluateGPOperator(37, pathProvider, aircraftState, args, 2);
    }
    // SIN
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(7, pathProvider, aircraftState, args, 1);
    }
    // ABS
    {
        double args[1] = {stack[sp-1]};
        sp -= 1;
        stack[sp++] = evaluateGPOperator(29, pathProvider, aircraftState, args, 1);
    }

    return applyRangeLimit(stack[0]);
}
