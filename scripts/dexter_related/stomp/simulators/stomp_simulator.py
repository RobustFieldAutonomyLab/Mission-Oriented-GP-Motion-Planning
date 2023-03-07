import kajiki
import sys
import shortuuid
import os
import shutil
import numpy as np
import subprocess
import pandas as pd

delete_folder = False

class TemplateProcessor():
    def __init__(self,template_file):
        with open (template_file, "r") as myfile:
            data=myfile.read()
            #print(data)
            self.TextTemplate = kajiki.TextTemplate(data)
        myfile.close()
        self.data = data

    def writefile(self, indict, outfilename):
        self.outfilename = outfilename
        #print("Pouring x into template \n",indict)
        data = self.TextTemplate(indict).render()
        #print("Poured Data")
        #print(data)
        #print(self.outfilename)
        with open (self.outfilename,"w") as outfile:
            #print("****WRITING FILE ", outfilename)
            outfile.write(data)
            outfile.flush()
        #print("done")


class MetricsReader():
    def __init__(self, metrics_file):
        self.metrics_file = metrics_file

    def read(self):
        self.outputs = {}
        with open(self.metrics_file,"r") as readfile:
            data = readfile.readlines()
            line = data[0]
            # #print(line)
            splits = line.split(":")
            self.outputs[splits[0]] = float(splits[1])
        path_df = pd.read_csv(self.metrics_file,skiprows=1)
        path = path_df.to_numpy()
        self.outputs['path'] = path
        return self.outputs


class STOMPSimulator():
    def __init__(self, **kwargs):
        if 'template_file' not in kwargs:
            # make this into a exter error - write DexterError exception
            print("DEXTER: ERROR (STOMP Simulator)- Template file cannot be found ")
            # print("**Exiting**")
            sys.exit()
        else:
            self.template_file = kwargs['template_file']
        if 'output_file' not in kwargs:
            self.output_file = "result.txt"
        else:
            self.output_file = kwargs['output_file']
        self.templateprocessor = TemplateProcessor(self.template_file)
        # copy the executable to the current directory
        mydir = os.path.dirname(os.path.realpath(__file__))
        if 'os' not in kwargs:
            print("DEXTER ERROR (STOMP Simulator)) - OS not specified in kwargs")
            sys.exit()
        execname = 'testSTOMP'# + kwargs['os']
        self.execfile = os.path.join(mydir, execname)
        self.cwd = os.getcwd()
        # dstfile = os.path.join(os.getcwd(), execname)
        # print("copying..%s \n %s "%(execfile,dstfile))
        # shutil.copyfile(execfile, dstfile)
        # shutil.copystat(execfile, dstfile)
        self.run_number = 0
        # print("done copying")

    def sim(self, x, **kwargs):
        # implementation of evaluation method in DesignSpace class expects an error if variables are arrays
        # to iterate over values of array - so raise type error if we get arrays in the input dictionary
        # (fortran code will fail otherwise)
        if any([isinstance(val, list) or isinstance(val, np.ndarray) for val in list(x.values())]):
            raise TypeError

        self.run_number = self.run_number+1
        print("Run# ", self.run_number)
        # print(x)

        # run_folder = 'Run_{0}'.format(self.run_number)
        run_folder = "dexStore_"+shortuuid.uuid()
        os.mkdir(run_folder)
        directory = os.path.join(self.cwd, run_folder)

        # we copy the executable to the current runfolder for each run - this is slow but guarantees thread safety
        shutil.copyfile(self.execfile, os.path.join(directory, 'exec_file'))
        shutil.copystat(self.execfile, os.path.join(directory, 'exec_file'))

        # tmpfilein = shortuuid.uuid()+".inp"
        # tmpfileout = shortuuid.uuid()+".out"
        tmpfilein = "tmp.yaml"
        tmpfileout = "result.txt"
        self.templateprocessor.writefile(x, os.path.join(directory, tmpfilein))
        bashCommand = "./exec_file" + " " + tmpfilein + " " + tmpfileout
        # print("Started Sim")
        p = subprocess.run(bashCommand, cwd=directory, shell=True)
        if p.returncode != 0:
            print('ERROR during execution of stomp code! Input:', x)
            return {'total_error': 10000000,
                    'path':[[0,0,0]],
                    'run_dir': run_folder}
        # print("Finished Sim")
        postprocess = MetricsReader(os.path.join(directory, self.output_file))
        y = postprocess.read()
        if len(y) == 0:
        #     for key in y:
        #         y[key] = y[key][0]
        #         try:
        #             y[key] = float(y[key])
        #         except ValueError:
        #             pass
        # else:
            y =     {'total_error': 10000000,
                     'path':[[0,0,0]],
                     'run_dir': run_folder}

        # delete folder
        if delete_folder:
            shutil.rmtree(directory)

        # if any([val is None for val in y.values()]):
        #     print('ERROR - None in metrics file! Input:', x, 'Output:', y)
        # print("output")
        # print(y)
        y['run_dir'] = run_folder
        return y


# m = MetricsReader("/home/rfal/Downloads/result.txt")
# print(m.read())
