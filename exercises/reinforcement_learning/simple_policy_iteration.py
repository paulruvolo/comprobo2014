#!/usr/bin/env python

"""
This Python script demonstrates the basics of the Policy Iteration algorithm
"""

import numpy as np
import pdb
from copy import copy

def evaluate_policy(policy,p,r,gamma):
	""" computes v^policy for each state by solving a system of linear equations given by:
		v^policy(x_i) = r(x_i,policy(x_i)) + gamma * \sum_{x'} p(x' | x_i, policy(x_i))*v^policy(x')
	"""
	# T represents the probability of transitioning between state x_t (row) and x_{t+1} (column) under the policy
	T = np.zeros((p.shape[1],p.shape[2]))
	# U represents the immediate reward values under the policy
	U = np.zeros((p.shape[1],1))
	for i in range(T.shape[0]):
		T[i,:] = p[policy[i],i,:]
		U[i] = r[i,policy[i]]
	v = np.linalg.solve(np.eye(T.shape[0]) - gamma*T,U)

	# compute the q function as well
	q = np.zeros((p.shape[1],p.shape[0]))
	for i in range(q.shape[0]):
		for j in range(q.shape[1]):
			q[i,j] = r[i,j] + gamma*p[j,i,:].T.dot(v)
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
	v,q = evaluate_policy(policy,p,r,gamma)
	print "value function is", v
	oldpolicy = policy
	# improve the policy
	policy = improve_policy(policy,q)
	converged = oldpolicy == policy
	print "new policy is ", policy