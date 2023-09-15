# f1tenth_rl
 f1tenth with reinforcement learning 

## Requirements
* Python 3.8
* Pip 22.0.8
* Pytorch (no need of cuda, works well with version 1.13.1)
* anaconda is an easy way to use pytorch 

# Installation
```bash
git clone https://github.com/CML-KU/f1tenth_rl.git --config core.autocrlf=input
cd f1tenth_rl
pip install --user -e gym
```

# Main
main at pkg/src/pkg folder
```bash
cd pkg/src/pkg
python dqn.py
```

You can add driver code there.

Basic code is for solo drivers and works with only one code

Other algorithms will be added.

# Cite

```
@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O�Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```