# f1tenth_FGM_reference_waypoint
 f1tenth with FGM REF WAYPOINT

## Requirements
* Python 3.8
* Pip 22.0.8

# Installation
```bash
git clone https://github.com/CSLab-GNU/f1tenth_bootcamp_GNU.git
cd f1tenth_bootcamp_GNU
pip install --user -e gym
```

# Main
```bash
cd pkg/src
python -m pkg.main
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