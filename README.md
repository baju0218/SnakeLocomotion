# Soft-body Locomotion with Deep Reinforcement Learning

## 연구 개요
1. Soft body를 움직이는 여러 방법 중, muscle 개념으로 soft body를 움직이도록 했을 때 이 개념이 효용성이 있는지 직접 검증
2. 이 muscle의 움직임을 강화학습(Deep Reinforcement Learning)을 통해 움직였을 때, 실제 유효한 동작을 발생시킬 수 있는지 검증

## 연구 결과
<p align="center">
   <a href="https://youtu.be/fQ-Zf69X4Sc?si=AOF7ZJ2sOWxZunb7" target="_blank">
      <img src="https://github.com/user-attachments/assets/9f1d771b-c8c9-4c1e-964e-3508528720cd" alt="Demo video" width="480" />
   </a>
   <br/>
   <a href="https://youtu.be/fQ-Zf69X4Sc?si=AOF7ZJ2sOWxZunb7">Full Video on YouTube</a>
</p>

- Soft body 모델(=snake)의 muscle을 적절히 활성화하여 목표 지점까지 강화학습(Deep Reinforcement Learning)을 통해 도달 가능한 것을 확인함

## Architecture

```
   ┌───────────────────────────────┐                                 ┌───────────────┐
   │ Python Gym                    │   action (muscle activation)    │ Python to C++ │
   │ (Deep Reinforcement Learning) │ ──────────────────────────────▶ │ (pybind11)    │
   └───────────────────────────────┘                                 └───────┬───────┘
           ▲                                                                 │
           │                                                                 │
           │                         Spinning Up PPO                         │
           │                                                                 │
           │                                                                 ▼
   ┌───────────────┐                           ┌─────────────────────────────┴───────┐
   │ C++ to Python │       state / reward      │ C++ Physics Engine                  │
   │ (pybind11)    │ ◀──────────────────────── │ (Soft Body, FEM, Muscle, Collision) │
   └───────────────┘                           └──────────────────────────┬──────────┘
                                                                          │
                                                                          │
                                                                          │
                                                                          │
                                                                          ▼
                                                               ┌─────────────────────┐
                                                               │ Real-time Rendering │
                                                               │ (OpenGL, GLFW)      │
                                                               └─────────────────────┘
```

## Tech Stack

- C++ Physical Engine
   - Soft body: FEM - [Vega FEM](https://viterbi-web.usc.edu/~jbarbic/vega/)
   - Muscle: 자체 개발
   - Collision: 자체 개발
- Render
   - OpenGL
   - GLFW - [GLFW](https://www.glfw.org/)
- Bridge
   - pybind11 - [pybind11](https://github.com/pybind/pybind11) (`simulation.pyd`)
- Deep Reinforcement Learning
   - Gym - [OpenAI Gym](https://github.com/openai/gym)
   - PPO - [OpenAI Spinning Up](https://github.com/openai/spinningup)

## Documentation

- Getting Started
  - See [`docs/Installation.txt`](docs/Installation.txt) for full setup details.
- Simulate & Render
  - See [`docs/Simulation.txt`](docs/Simulation.txt) and [`docs/Render.txt`](docs/Render.txt) for API reference and key bindings.
- Deep Reinforcement Learning
  - See [`docs/Deep Learning.txt`](docs/Deep%20Learning.txt) for running Deep Reinforcement Learning.

## References

**주요 논문**
- *Soft Body Locomotion*
    - Tan & Turk, SIGGRAPH 2012
    - [[PDF]](https://faculty.cc.gatech.edu/~turk/my_papers/soft_body_locomotion.pdf)

**관련 연구**
- *SoftCon: Simulation and Control of Soft-Bodied Animals with Biomimetic Actuators*
    - Min, Won, Lee, Park, Lee, SIGGRAPH Asia 2019 (SNU MRL)
    - [[Project]](https://mrl.snu.ac.kr/publications/ProjectSoftCon/SoftCon.html)
    - [[Code]](https://github.com/seiing/SoftCon)
