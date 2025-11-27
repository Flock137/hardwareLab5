# Lab Session: PID Tuning for Brushless DC Motor Speed Control

**Course:** Intelligent Physical System

**Lab Duration:** 2 hours

**Prerequisite Knowledge:** Students should be familiar with PID control basics, MATLAB, and BLDC motor dynamics.

## Objective

This lab session aims to provide hands-on experience in tuning Proportional Integral Derivative (PID) controllers for the speed control of a Brushless DC Motor (BLDC). Students will explore both manual and automatic PID tuning methods, comparing their performance and understanding the impact of each gain parameter on the motor's response.

## Overview

Students will use two MATLAB scripts:

1. **Manual PID Tuning**: Students will iteratively adjust the $K_p$, $K_i$, $K_d$ values and observe the step response of the motor.

2. **Automatic PID Tuning**: Students will explore automatic tuning techniques, including Ziegler-Nichols and MATLAB's PID Tuner, to achieve optimal performance.

## Equipment and Software

- MATLAB (with Control System Toolbox installed).
- Provided MATLAB scripts for manual and automatic PID tuning:
  - **Manual_PID_DC_Tunning.m**
  - **Auto_PID_DC_Tunning.m**

## Lab Tasks

The lab is divided into two main tasks, focusing on manual and automatic PID tuning.

## Task 1: Manual PID Tuning

### Step 1: Initialize Parameters

- Load the **Manual_PID_DC_Tunning.m** script in MATLAB.
- Run the script to observe the initial step response with the default PID gains ($K_p = 1$, $K_i = 0$, $K_d = 0$)

### Step 2: Understand the Response

- Observe the initial response and identify characteristics such as **rise time**, **overshoot**, **settling time**, and **steady-state error**.

### Step 3: Iteratively Adjust PID Gains

- Students will iteratively change $K_p$, $K_i$, $K_d$ values as prompted by the script.
- Adjust each gain parameter one at a time:
  - Start by increasing $K_p$ to observe its effect on the rise time and overshoot.
  - Next, increase $K_i$ to reduce the steady-state error but be cautious of integral windup.
  - Finally, adjust $K_d$ to reduce overshoot and improve system damping.

### Step 4: Optimize the Response

- Continue tuning until the step response meets the following criteria:
  - **Minimal overshoot**
  - **Fast settling time**
  - **Reduced steady-state error**

### Step 5: Document Results

- Record the final PID gains that achieve the best performance.
- Plot and annotate the final step response, highlighting the improvements made through manual tuning.

## Task 2: Automatic PID Tuning

### Step 1: Run the Automatic PID Tuning Script

- Load the **Auto_PID_DC_Tunning.m** script in MATLAB.
- The script implements two automatic tuning methods:
  1. **Ziegler-Nichols Method**
  2. **MATLAB's PID Tuner**
- Run the script and observe the response for each tuning method.

### Step 2: Analyze the Ziegler-Nichols Method

- Understand the Ziegler-Nichols method, which uses the estimated ultimate gain ($K_u$) and ultimate period ($T_u$)
- Observe how the automatically calculated gains ($K_p$, $K_i$, $K_d$) compare with the manually tuned gains.
- Record the Ziegler-Nichols PID gains and plot the corresponding step response.

### Step 3: Analyze MATLAB's PID Tuner

- MATLAB's PID Tuner provides an optimized set of gains using built-in tuning algorithms.
- Observe the step response and compare it with the Ziegler-Nichols and manually tuned responses.

### Step 4: Compare Results

- Compare the performance of all three tuning methods (manual, Ziegler-Nichols, and MATLAB's PID Tuner) based on:
  - **Rise time**
  - **Overshoot**
  - **Settling time**
  - **Steady-state error**

### Step 5: Document Observations

- Create a table summarizing the PID gains and performance metrics for each method.
- Analyze which tuning method provided the best performance and discuss potential tradeoffs.

## Lab Report Requirements

Students must submit a report that includes:

1. **Introduction**: Brief explanation of the purpose of the lab and an overview of PID control.

2. **Methodology**: Description of the manual and automatic tuning processes, including the parameters adjusted.

3. **Results**:
   - Plots of step responses for each tuning method.
   - Comparison table of PID gains and performance metrics.
   - Figures and observations for task 1, task 2, and task 3.
   - Your own version of the same figure in task 3. Shows both axes.

4. **Discussion**:
   - Analysis of the effect of each PID gain on the BLDC motor's speed response.
   - Explain your understanding of transfer functions, PID's gains role, Ziegler-Nichols method.
   - Comparison of the tuning methods, discussing advantages and disadvantages of each.

5. **Conclusion**: Summary of findings and lessons learned about PID tuning.

## Learning Outcomes

By the end of this lab session, students will be able to:

- Understand the role of each PID gain in controlling a BLDC motor.
- Manually tune a PID controller to improve system performance.
- Apply automatic tuning methods and compare their effectiveness.
- Analyze and optimize control performance for a given setpoint.


---

## Personal notes

## Manual tuning 

### Tuning K_p
Goal: Find a $K_p$ that gives you a reasonably fast response without excessive overshoot (maybe 10-20% overshoot is okay for now).
Answer: K_p of 2.5 seems good to me 

### Tuning K_i
Goal: Find smallest K_i that eliminates steady-state error without making the system sluggish or oscillatory.
Choose value of 1.

### Tuning K_d
Goal: Find value that minimizes overshoot while keeping response reasonably fast.
Pick 0.5


### Auto Tuning 
Ziegler-Nichols Tuning...
MATLAB PID Tuner...
 
Step Response Characteristics for Ziegler-Nichols Tuning:

| Rise Time | Settling Time | Overshoot | Steady State Error |
|-----------|---------------|-----------|-------------------|
| 0.36534   | 2.8373        | 31.427    | 0.00098278        |

Step Response Characteristics for PID Tuner:

| Rise Time | Settling Time | Overshoot | Steady State Error |
|-----------|---------------|-----------|-------------------|
| 0.27957   | 1.0919        | 15.767    | 2.1183            |

 
Automatic PID Tuning Completed.
 
Ziegler-Nichols PID Gains: Kp = 6, Ki = 7.5, Kd = 1.2
 
PID Tuner Gains: Kp = 10.7949, Ki = 0, Kd = 1.6278


## All data in a table 


| Method          | $K_p$   | $K_i$<br> | $K_d$<br> | Rise Time (s) | Overshoot (%) | Settling Time (s) | Steady State Error (rad/s) |
| --------------- | ------- | --------- | --------- | ------------- | ------------- | ----------------- | -------------------------- |
| Manual          | 2.5     | 1         | 0.5       | 0.74          | 17.72         | 3.98              | 0.26                       |
| Ziegler-Nichols | 6       | 7.5       | 1.2       | 0.36534       | 31.427        | 2.8373            | 0.00098278                 |
| MATLAB Tuner    | 10.7949 | 0         | 1.6278    | 0.27957       | 15.767        | 1.0919            | 2.1183                     |






