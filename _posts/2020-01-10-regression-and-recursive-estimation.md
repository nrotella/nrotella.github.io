---
layout: post
title: "Regression and Recursive Estimation"
author: "Nick Rotella"
categories: journal
tags: [mathematics, controls, estimation]
image: 
---

Up until this point, we've kept things pretty abstract. How does solving $$Ax=b$$ for the least squares solution relate to estimation? Let's start with an application you've probably used before - fitting a line to some noisy experimental data, also known as **linear regression**.

* Table of contents:
{:toc}

# Linear Regression

Let's say you have a bunch of data points in two dimensions (for simplicity, though the result is the same for any number of dimensions). This data could be anything - for example, number of auto accidents vs miles driven, cost of soy vs annual rainfall, or even motion of planets vs time (as was the original application of linear regression) - anything in which you hope to find some trend in the data. 
