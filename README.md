# Soft Body Locomotion with Deep Reinforcement Learning

### 연구 개요)
1. Soft body를 움직이는 여러 방법 중, `Muscle` 개념으로 Soft body를 움직이도록 했을 때 이 개념이 효용성이 있는지 직접 검증
2. 이 Muscle의 움직임을 `강화학습(Deep Reinforcement Learning)`을 통해 움직였을 때, `실제 유효한 동작`을 발생시킬 수 있는지 검증

### 연구 결과)

- [Link to Youtube](https://youtu.be/fQ-Zf69X4Sc?si=AOF7ZJ2sOWxZunb7)
- <a href="https://youtu.be/fQ-Zf69X4Sc?si=AOF7ZJ2sOWxZunb7" target="_blank"> 
  <img width="300" alt="Youtube" src="https://github.com/user-attachments/assets/7c9d52d3-ce70-425f-bd5d-934e179e6438" /> 
</a> 

**결론**
> Snake 모델(=Soft body)의 Muscle을 적절히 활성화하여 목표 지점까지 강화학습(Deep Reinforcement Learning)을 통해 도달 가능한 것을 확인함

## Architecture

<img width="600" alt="Architecture" src="https://github.com/user-attachments/assets/a0aa3e05-2732-49a7-8840-b3751cf3ee5e" />

## Tech Stack

- C++ Physical Engine
  - Vega FEM: https://viterbi-web.usc.edu/~jbarbic/vega
  - Muscle: 자체 개발
  - Collision: 자체 개발
- 3D Rendering
  - OpenGL
  - GLFW
- C++/Python Bridge
  - Pybind11: https://github.com/pybind/pybind11
- Deep Reinforcement Learning
  - Gym: https://github.com/openai/gym

## Documentation

- Getting started
  - See `docs/Installation.txt` for full setup details

- How to use
  - See `docs/Simulation.txt` and `docs/Render.txt` for API reference and key bindings
  - See `docs/Deep Learning.txt` for running deep reinforcement learning

## References

- **Soft Body Locomotion (SIGGRAPH 2012)**
  - Jie Tan, Greg Turk, Karen Liu (Georgia Institute of Technology)
  - https://faculty.cc.gatech.edu/~turk/my_papers/soft_body_locomotion.pdf

- **SoftCon: Simulation and Control of Soft-Bodied Animals with Biomimetic Actuators (SIGGRAPH Asia 2019)**
   - Sehee Min, Jungdam Won, Seunghwan Lee, Jungnam Park, Jehee Lee (Seoul National University)
   - https://mrl.snu.ac.kr/publications/ProjectSoftCon/SoftCon.html
