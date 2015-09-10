import subprocess
import numpy as np

def spearmint_python_driver(nsimulations, speed_delta_count, speed_delta_jump, k_c, k_s, k_v):

  bashCommand = 'julia /home/tim/.julia/v0.3/AutomotiveDrivingModels/scripts/risk_estimation_policy_opt/run_risk_estimation_policy.jl' \
                ' nsimulations ' + str(nsimulations) + ' speed_delta_count ' + str(speed_delta_count) + ' speed_delta_jump ' + str(speed_delta_jump) + ' k_c ' + str(k_c) + ' k_s ' + str(k_s) + ' k_v ' + str(k_v)

  print bashCommand

  process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
  output = process.communicate()[0]
  result = -float(output)

  print 'Result = %f' % result

  return {'main' : result}

def main(job_id, params):
  print 'Anything printed here will end up in the output directory for job #%d' % job_id
  print params

  log10_k_c = params['log10_k_c'][0]
  k_c = np.exp(log10_k_c * np.log(10))

  return spearmint_python_driver(params['nsimulations'][0], params['speed_delta_count'][0], params['speed_delta_jump'][0], k_c, params['k_s'][0], params['k_v'][0])