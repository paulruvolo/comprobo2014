#!/usr/bin/env python

"""
This Python script demonstrates the basics of the Monte-Carlo Policy Iteration algorithm with exploring starts
"""

import numpy as np
import pdb
from copy import copy

def evaluate_policy_montecarlo(policy,p,r,gamma,n):
	""" computes v^policy and q^policy using monte-carlo evaluation
	"""
	# compute the q function as well
	q = np.zeros((p.shape[1],p.shape[0]))
	v = np.zeros((p.shape[1],1))
	for i in range(q.shape[0]):
		for j in range(q.shape[1]):
			returns = []
			for trial in range(n):
				x = i
				u = j
				rsum = r[x,u]
				probs = p[u,x,:]

				# 100 is an arbitrary threshold where gamma**100 is sufficiently low
				for t in range(1,100):
					probs = p[u,x,:]
					x = np.random.choice(np.arange(p.shape[1]),p=probs)
					u = policy[x]
					rsum += r[x,u]*gamma**t
				returns.append(rsum)
			q[i,j] = sum(returns)/n
	for i in range(q.shape[0]):
		v[i] = q[i,policy[i]]
	return v,q

def improve_policy(policy,q):
	newpolicy = copy(policy)
	for i in range(len(policy)):
		newpolicy[i] = np.argmax(q[i,:])
	return newpolicy

# encodes p(x_{t+1} | x_t, u_t), the first dimension is u_t, next is x_t, and the third is x_{t+1}
p = np.array([[[0.6, .25, .15],
			   [0.0, 1.0, 0.0],
			   [0.3, 0.0, 0.7]],
			  [[0.1, .8, .1],
			   [0.0, 0.0, 1.0],
			   [0.0, 0.5, 0.5]]])

# encodes r(x_t, u_t), the first dimension is x_t, and the second is u_t
r = np.array([[0.0, 0.0],
			  [0.0, 0.0],
			  [1.0, 1.0]])

# the discount factor for the MDP
gamma = 0.9

# initialize the policy (at first always execute action 0)
policy = [0, 0, 0]
print "policy is ", policy

converged = False
while not(converged):
	# evaluate the policy
	v,q = evaluate_policy_montecarlo(policy,p,r,gamma,100)
	print "value function is", v
	oldpolicy = policy
	# improve the policy
	policy = improve_policy(policy,q)
	converged = oldpolicy == policy
	print "new policy is ", policy