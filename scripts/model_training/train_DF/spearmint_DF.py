import os
import subprocess
import numpy as np

def spearmint_DF(params):

  directory = os.path.dirname(os.path.abspath(__file__))

  ntrees = params['ntrees'][0]
  max_depth = params['max_depth'][0]
  min_samples_split = params['min_samples_split'][0]
  min_samples_leaves = params['min_samples_leaves'][0]
  min_split_improvement = params['min_split_improvement'][0]
  partial_sampling = params['partial_sampling'][0]

  bashCommand = 'julia ' + directory + '/train_DF.jl' \
                ' ntrees ' + str(ntrees) + ' max_depth ' + str(max_depth) + ' min_samples_split ' + str(min_samples_split) + ' min_samples_leaves ' + str(min_samples_leaves) + ' min_split_improvement ' + str(min_split_improvement) + ' partial_sampling ' + str(partial_sampling)
  print bashCommand

  process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
  output = process.communicate()[0]
  result = -float(output)

  print 'Result = %f' % result

  return {'main' : result}

def main(job_id, params):
  print 'Anything printed here will end up in the output directory for job #%d' % job_id
  print params

  return spearmint_DF(params)
