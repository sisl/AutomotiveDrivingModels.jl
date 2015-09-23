import os
import subprocess
import numpy as np

def spearmint_BN(params):

  directory = os.path.dirname(os.path.abspath(__file__))

  ncandidate_bins = params['ncandidate_bins'][0]

  bashCommand = 'julia ' + directory + '/train_BN.jl' \
                ' ncandidate_bins ' + str(ncandidate_bins) + ' preoptimize_indicator_bins ' + str(preoptimize_indicator_bins)
  print bashCommand

  process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
  output = process.communicate()[0]
  result = -float(output)

  print 'Result = %f' % result

  return {'main' : result}

def main(job_id, params):
  print 'Anything printed here will end up in the output directory for job #%d' % job_id
  print params

  return spearmint_BN(params)
