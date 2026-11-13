# 11 — Learning roadmap

How learned walking is added in version 7, and what earlier versions do to keep
that door open. Nothing here is built before version 7.

## What "learning to walk" means

A small neural network, the **policy**, is trained to map what the robot senses to
what its joints should do. Training is trial and error: thousands of simulated
robots try, a score rewards good walking, and the network is adjusted toward
higher scores. This is reinforcement learning.

Training happens on a PC in simulation. Only the finished policy runs on the robot.

## Why it is not version 1

| Reason | Detail |
|---|---|
| It is not needed for balance | Six legs give static stability; geometry is enough to walk |
| It needs an accurate model | A policy trained on a wrong model fails on the real robot |
| It needs feedback servos | The policy must sense joint positions as they really are |
| It is hard to debug | When a learned walker fails, the cause is not visible in the code |
| The programmed walker is its reference | Without a baseline there is nothing to compare it against |

## What earlier versions do to prepare

| Version | Preparation |
|---|---|
| 0 | Leg maths kept free of ROS, so the training environment can import it |
| 1 | MuJoCo chosen as the simulator; the same model file is reused for training |
| 1.5 | Masses and inertias from CAD |
| 2 | Servo behaviour measured and fitted into the model |
| 3 | Real walking data logged for comparison with the simulation |
| 4 | Tilt sensor available: the policy's most important input after joint angles |
| 5 | Foot contact available |

Choosing smart servos with feedback is the hardware decision that keeps this
possible. See [decision 0004](decisions/0004-smart-bus-servos.md).

## The plan for version 7

### Policy inputs and outputs

| Inputs (what the policy sees) | Source |
|---|---|
| 18 joint angles and speeds | Servo feedback |
| Direction of gravity in the body frame | Tilt sensor |
| Body rotation rate | Tilt sensor |
| Velocity command | `/cmd_vel` |
| The previous action | The policy's own last output |
| Optionally, foot contacts | Contact module |

| Output | Detail |
|---|---|
| 18 joint target offsets | Added to the neutral stance, then sent to the same joint controller the programmed walker uses |

Run rate: 50 per second, the same as the programmed walker.

### Training setup

| Element | Plan |
|---|---|
| Simulator | MuJoCo, using a runtime that simulates thousands of robots in parallel |
| Framework | A MuJoCo-based robot-learning framework. Candidates at the time of writing: mjlab, MuJoCo Playground. Re-evaluated when version 7 starts, because this field moves quickly |
| Algorithm | PPO, the standard choice for legged robots |
| Hardware | The development PC's NVIDIA RTX 5060 Ti (16 GB). Training tools use the card from inside WSL 2; if that proves troublesome, training runs under dual-boot Ubuntu or on a rented cloud machine |

### Reward: what counts as good walking

| Term | Encourages |
|---|---|
| Following the commanded velocity | Going where it is told |
| Staying upright and at the right height | A stable body |
| Low joint torque and smooth actions | Efficiency; less wear and heat |
| Feet not slipping | Clean steps |
| Penalty for body or knees touching the ground | No crawling |
| Optional: similarity to the programmed gait | A sensible starting style |

### Crossing from simulation to the real robot

A policy overfits to the exact simulator unless training is deliberately varied.
During training, these are randomized for each simulated robot:

| Randomized | Range chosen from |
|---|---|
| Body and link masses | Weighing the real robot, ± a margin |
| Servo strength and speed | The version 2 measurements, ± a margin |
| Ground friction | Different floor surfaces |
| Sensor noise and delay | Measured on the real sensors |
| Command delay | Measured bus timing |
| Random pushes | To teach recovery |

### Running on the robot

| Step | Detail |
|---|---|
| Export | The trained network is saved in ONNX format |
| Run | A small node on the Pi 5 loads it with an ONNX runtime and runs it on the processor. The network is small; the Pi 5 has ample capacity |
| Interface | Same topics as the programmed walker. Selected with `walker:=policy` |
| Safety | Joint limits, torque limits and tip-over detection stay active underneath. The programmed walker remains available as a fallback |

## Options in order of ambition

| Option | What is learned | Risk |
|---|---|---|
| 1. Tuning | Only the programmed walker's parameters (cycle time, step height, stride) | Low |
| 2. Residual | Small corrections added on top of the programmed walker's output | Medium |
| 3. Full policy | Everything, from sensors to joint targets | Higher |

Version 7 starts with option 2 or 3 depending on how well simulation and reality
agree by then. Option 1 can be done at any time after version 1 as a warm-up.

## Beyond walking

Once the training pipeline exists, the same method can learn rough-terrain walking,
recovery from a fall, energy-efficient gaits, or walking with a damaged leg.
