from pyperplan.planner import _parse, _ground
from pyperplan.heuristics.landmarks import get_landmarks
import os
import tarfile
import pandas as pd

class ExtractLandmarks():
    '''
    self.counter - total number of goals
    self.domainfile - location of the domain file
    self.goals - list of goals
    self.landmarks - list of landmarks generated from goals
    self.template - template of task pddl file
    '''
    def __init__(self, *args):
        '''
        Constructs landmarks out of given domain file, goals list and task template pddl.
        '''
        self.landmarks = []
        if len(args) == 1:
            self.__unpackTar(*args)
        elif len(args) == 3:
            self.__unpackFiles(*args)
        else:
            raise TypeError("Incorrect number of arguments.")

    def __unpackFiles(self, domaindir, goalsdir, templatedir):
        '''
        Loads the necessary resources into class variables. This function is called when
        three arguments are given.
        '''
        self.domainfile = os.path.abspath(domaindir)
        with open(goalsdir) as goalsfile:
            self.goals = goalsfile.read().splitlines()
        with open(templatedir) as templatefile:
            self.template = templatefile.read()
        self.__populate()

    def __unpackTar(self, tardir):
        '''
        Loads the necessary resources into class variables. This function is called when
        one argument is given.
        '''
        with tarfile.open(tardir, 'r:bz2') as tar:
            tar.extract('domain.pddl', path="temp")
            self.domainfile = os.path.abspath('temp/domain.pddl')
            self.goals = tar.extractfile('hyps.dat').read().decode('utf-8').splitlines()
            self.template = tar.extractfile('template.pddl').read().decode('utf-8')
        self.__populate()

    def __populate(self):
        '''
        Creates task files for each goal using the template, 
        and uses these task files to extract landmarks.
        '''
        for i in range(len(self.goals)):
            dirname = f"temp/task{i}.pddl"
            task = self.template.replace("<HYPOTHESIS>", self.goals[i])
            with open(dirname, "w") as create:
                create.write(task)
            parsed = _parse(self.domainfile, dirname)
            task = _ground(parsed)
            landmarks = get_landmarks(task)
            self.landmarks.append(landmarks)

    def getLandmark(self, landmark = None):
        '''
        Returns a specific landmark, or returns all landmarks depending 
        on whether an argument is given.
        '''
        return self.landmarks if landmark is None else self.landmarks[landmark]

    def getGoal(self, goal = None):
        '''
        Returns a specific goal, or returns all goals depending 
        on whether an argument is given.
        '''
        return self.goals if goal is None else self.goals[goal] 

    def table(self):
        # Return a table of the landmark intersection for each pair of goals.
        data = [[a.intersection(b) for a in self.landmarks] for b in self.landmarks]
        return pd.DataFrame(data)

    def intersection(self, *args):
        '''
        '''
        return set.intersection(*[self.landmarks[i] for i in args] if len(args) else self.landmarks)

    def path(self, state, landmark):
        pass
    
if __name__ == "__main__":
    # Defining constants
    EXPERIMENTS_DIR = 'experiments/raw'
    EXPERIMENTS_TAR_DIR = 'experiments/tar'
    RESULTS_DIR = 'results'

    # Iterate through each problem set
    for _, dirs, _ in os.walk(EXPERIMENTS_DIR):
        for dname in dirs:
            print(f"Extracting landmarks for {dname}")
            domaindir = f"{EXPERIMENTS_DIR}/{dname}/domain.pddl"
            hypsdir = f"{EXPERIMENTS_DIR}/{dname}/hyps.dat"
            templatedir = f"{EXPERIMENTS_DIR}/{dname}/template.pddl"
            extraction = ExtractLandmarks(domaindir, hypsdir, templatedir)
            print(extraction.intersection(1,2,3,0))

    # For TAR files
    '''
    for _, _, files in os.walk(EXPERIMENTS_TAR_DIR):
        for tarfname in files:
            extraction = ExtractLandmarks(f"{EXPERIMENTS_TAR_DIR}/{tarfname}")
    '''


