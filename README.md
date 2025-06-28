# SimplifiedMDMPC

## Overview

This repository contains the code for the research article on a simplified
distributed model predictive control (DMPC) scheme. The research focuses on
providing a computationally efficient control solution that is particularly
effective for systems with complex, nonlinear dynamics. The work presented here
was used to generate the results and figures discussed in the associated
publication.

The core of this project is to demonstrate a control strategy that simplifies
the traditional DMPC approach by using a fixed bank of linear models, which are
selected based on the current operating region of the system. This
simplification leads to a significant reduction in computational load without a
major sacrifice in control performance, making it suitable for real-world
applications where computational resources may be limited.

## Associated Research Article

The code in this repository was used to produce the findings detailed in the
following academic paper:

**Title:** A simplified multi-model-based model predictive control (MDMPC)
scheme **Journal:** _Chemical Engineering Science_ **Citation:** Chilin, D.,
Liu, J., & Biegler, L. T. (2012). A simplified multi-model-based model
predictive control (MDMPC) scheme. _Chemical Engineering Science_, 75, 241-249.
**DOI:** [10.1016/j.ces.2012.03.045](https://doi.org/10.1016/j.ces.2012.03.045)

You can find the abstract at ScienceDirect:
[https://www.sciencedirect.com/science/article/abs/pii/S0009250912002965](https://www.sciencedirect.com/science/article/abs/pii/S0009250912002965)

## Key Features

- Implementation of a simplified DMPC algorithm.
- Code for simulating and testing the control strategy on a CSTR model.
- Scripts for generating plots and figures from the simulation data.
- Data files from the simulations discussed in the research paper.

## Visualizations from the Study

Below are some results from the re-configuration simulations presented in the
2011 data folder, specifically for the `F6=0.5 Reconfiguration` case.

### Concentration Profile

<<<<<<< HEAD
![Concentration Profile](<./data/2011/F6=0.5\ Reconfiguration/Concentration.svg>)
=======
![Concentration Profile](./data/2011/F6%3D0.5%20Reconfiguration/Concentration.svg)

![Alt text](./data/2011/F6%3D0.5%20Reconfiguration/Concentration.svg)
<img src="./data/2011/F6\=0.5\ Reconfiguration/Concentration.svg">

> > > > > > > d4b7d0291bec38b6f9a053565d8cf72291e3e52f

### Controller Action

![Controller Action](https://raw.github.com/davidchilin/SimplifiedMDMPC/refs/heads/master/data/2011/F6_0.5_Reconfiguration/Controller.svg?sanitize=true)

### Temperature Profile

![Temperature Profile]("https://rawgithub.com/davidchilin/SimplifiedMDMPC/refs/heads/master/data/2011/F6_0.5_Reconfiguration/Temperature.svg")
<img src="https://rawgithub.com/davidchilin/SimplifiedMDMPC/refs/heads/master/data/2011/F6_0.5_Reconfiguration/Temperature.svg">

![Alt text](https://raw.github.com/potherca-blog/StackOverflow/master/question.13808020.include-an-svg-hosted-on-github-in-markdown/controllers_brief.svg)
<img src="https://raw.github.com/potherca-blog/StackOverflow/master/question.13808020.include-an-svg-hosted-on-github-in-markdown/controllers_brief.svg">

![Alt text](https://raw.github.com/potherca-blog/StackOverflow/master/question.13808020.include-an-svg-hosted-on-github-in-markdown/controllers_brief.svg?sanitize=true)
<img src="https://raw.github.com/potherca-blog/StackOverflow/master/question.13808020.include-an-svg-hosted-on-github-in-markdown/controllers_brief.svg?sanitize=true">

![Alt text](https://rawgithub.com/potherca-blog/StackOverflow/master/question.13808020.include-an-svg-hosted-on-github-in-markdown/controllers_brief.svg)
<img src="https://rawgithub.com/potherca-blog/StackOverflow/master/question.13808020.include-an-svg-hosted-on-github-in-markdown/controllers_brief.svg">

## Dependencies

_(List any required software, libraries, or toolboxes needed to run the code,
for example:)_

- MATLAB R2011a or later
- COIN-OR IPOPT

## License

This project is licensed under the MIT License - see the
[LICENSE.md](LICENSE.md) file for details or original license of respective
software (ie. ipopt EPL2.0)
