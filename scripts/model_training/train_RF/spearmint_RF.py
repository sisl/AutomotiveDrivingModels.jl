import os
import subprocess
import numpy as np

def spearmint_RF(params):

  directory = os.path.dirname(os.path.abspath(__file__))

  ntrees = params['ntrees'][0]
  max_depth = params['max_depth'][0]

  bashCommand = 'julia ' + directory + '/spearmint_RF.jl' \
                ' ntrees ' + str(ntrees) + ' max_depth ' + str(max_depth)

  print bashCommand

  process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
  output = process.communicate()[0]
  result = -float(output)

  print 'Result = %f' % result

  return {'main' : result}

def main(job_id, params):
  print 'Anything printed here will end up in the output directory for job #%d' % job_id
  print params

  return spearmint_RF(params)
