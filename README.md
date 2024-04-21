# ME599 Data-Driven Methods for Control Systems Final Project

## Data Driven Control of Ornithopter Dynamics using Koopman Operator Theory
### Team Member: Hsin Yu Wei, Ting-Hao Ling

This repo contains all source files to replicate the results presented

To Simulate
1. PID, run **PID.m**
2. Linear MPC, run **LMPC.m**
   - In line **4-6**, toggle online regression and integral controller on/off as desired
4. Nonlinear MPC, run **NLMPC**
5. MPC in original space, run **baseline_NLMPC**
   - This may take around 2 hr to run

Data training procedure
1. To get training data, run **data_gathering.m**
   - This may take around 1 hr with parallel computing 
3. To train linear Koopman operator, run **linear_system_fitting.m**
4. To train nonlinear Koopman operator, run **nonlinear_system_fitting**

To plot all results, run **plot_results.m**
