import functools
from pyperplan.planner import _parse, _ground
from pyperplan.search.a_star import astar_search
from pyperplan.heuristics.landmarks import *
from src.pyperplan.search.a_star import astar_search as astar_search_custom
from pyperplan.heuristics.blind import *
import os

class ExtractLandmarks():
    '''
    self.domainfile - location of the domain file
    self.template - template of task pddl file
    self.landmarkHeur - List of 

    self.goals - list of goals
    self.realgoal - the actual goal
    self.observations - observations
    self.landmarks - list of landmarks generated from goals

    self.debug - whether to print debug comments
    '''
    #################
    ### VARIABLES ###
    #################
    TEMP_DIR = os.path.join(os.path.dirname(__file__), "temp") # Location of temp folder

    ###################################
    ### INITIALIZATION OF LANDMARKS ###
    ###################################
    def __init__(self, *args, debug = False):
        '''
        Constructs landmarks out of given domain file, goals list and task template pddl.
        '''
        self.debug = debug
        self.landmarks = []
        if len(args) == 1:
            pass
             # self.__unpackTar(*args)
        elif len(args) == 4:
            self.__unpackFiles(*args)
        else:
            raise TypeError("Incorrect number of arguments.")

    def __unpackFiles(self, domaindir, hypsdir, realhypdir, templatedir):
        '''
        Loads the necessary resources into class variables. This function is called when
        three arguments are given.
        '''
        print(f"Getting landmarks for {domaindir} {hypsdir} {templatedir}\n")
        self.domainfile = os.path.abspath(domaindir)
        with open(hypsdir) as goalsfile:
            self.goals = goalsfile.read().splitlines()
        with open(realhypdir) as realhypfile:
            self.realgoal = self.goals.index(realhypfile.readline())
        with open(templatedir) as templatefile:
            self.template = templatefile.read()

        # DEBUG
        self.__output(
            '# List of Goals parsed:', 
            *[f"{i} : {a}" for i, a in enumerate(self.goals)]
        )
        self.__output(
            '# Real Goal parsed:', 
            f"{self.realgoal} : {self.goals[self.realgoal]}"
        )

        self.__populate()
    
    def __populate(self):
        '''
        Creates task files for each goal using the template, 
        and uses these task files to extract landmarks.
        '''
        for i in range(len(self.goals)):
            dirname = self.temp(f"task{i}.pddl")
            task = self.template.replace("<HYPOTHESIS>", self.goals[i])
            with open(dirname, "w") as create:
                create.write(task)
            parsed = _parse(self.domainfile, dirname)
            task = _ground(parsed)
            landmarks = get_landmarks(task)
            self.landmarks.append(landmarks)
            
        self.__output(
            '# List of Landmarks calculated:',
            *[f"{i} : {self.goals[i]} : {a}\n" for i, a in enumerate(self.landmarks)]
        )

    ############################################
    ### FUNCTIONS INTERACTING WITH LANDMARKS ###
    ############################################
    def table(self):
        '''Return a table of the landmark intersection for each pair of goals.
        '''
        data = [[a.intersection(b) for a in self.landmarks] for b in self.landmarks]
        # result = pd.DataFrame(data)
        self.__output(f"Finding intersection table between all landmarks", result= data)
        return data

    def intersection(self, *args):
        ''' Finds the intersections of landmarks between given goals
        '''
        result = set.intersection(*[self.landmarks[i] for i in args] if len(args) else self.landmarks)
        self.__output(f"Finding intersection of landmarks between goals: {*args,}", result = result)
        return result
    
    def reducing(self):
        ''' Trying something new
        '''
        initialTask = _ground(_parse(self.domainfile, self.temp("task0.pddl")))

        def toLandmark(acc, landmark):
            ''' Given a task and a landmark, calculate the number of steps to achieve this landmark
            and calculate the end state after traversing the path.
            '''
            task, steps = acc
            self.__output(f"\n# Finding path to {landmark}")

            task.goals = frozenset({landmark})
            heuristic = LandmarkHeuristic(task)
            actual = astar_search_custom(task, heuristic, return_state=True) # Patrick's editted code
            path = astar_search(task, heuristic) # Generate a path
            # Applying these ops to the state
            for op in path:
                steps += 1
                print(f"Current State: {task.initial_state}")
                print(f"Applying step {steps}: {op}")
                task.initial_state = op.apply(task.initial_state)
            assert task.initial_state == actual # Making sure the final state is correct
            return (task, steps)

        landmarkIntersection =  [i.intersection(self.landmarks[self.realgoal]) for i in self.landmarks]
        landmarkIntersection[self.realgoal] = {} # Intersection with self to empty set
        self.__output(
            "# Intersection of goals with the real goal",
            *[f"{i}: {a} " if i != self.realgoal else "" for i, a in enumerate(landmarkIntersection)])

        # How do I use these to find a deceptive path??
        # Attempt: the more landmarks there are in common, the more similiar the goals are?
        landmarkSet = max(landmarkIntersection, key=len) # Result has a list of landmarks
        self.__output(
            "# The intersection with the largest number of landmarks",
            *[f"{i}: {a} " for i, a in enumerate(landmarkSet)])
        # TODO: Find a way to order landmarks to reduce path length / cost

        def calculate_score(task, landmark):
            task.goals = frozenset({landmark})
            test = LandmarkHeuristic(task)
            h = test.landmarks - test.task.initial_state
            print(f"Landmark: {landmark}, Score: {len(h)}")
            return len(h)
        
        sorted_h = sorted(landmarkSet, key = lambda landmark: calculate_score(initialTask, landmark))
        print(f"Sorted based on score: {sorted_h}")
        return functools.reduce(toLandmark, sorted_h, (initialTask, 0))

    ########################
    ### USEFUL FUNCTIONS ###
    ########################

    def __output(self, *lines, result = None ):
        ''' Function to make pretty outputs.
        '''
        if self.debug:
            for l in lines:
                print(l)
            if result:
                print(f"Result: {result}")
            print("-----------------\n")

    def temp(self, name):
        ''' Returns an absolute directory to the 
        '''
        return os.path.join(self.TEMP_DIR, name)

    def setDebug(self, debugMode = True):
        '''
        Whether to have outputs.
        '''
        self.debug = debugMode

    ##################################################
    ### UNUSED STUFF WHICH MIGHT BE HANDY LATER ON ###
    ##################################################
    """
    UNUSED, probably does not work currently
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
    """

    """ UNUSED
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
    """
    
if __name__ == "__main__":
    DIR = os.path.dirname(__file__)
    # Defining constants
    EXPERIMENTS_DIR = os.path.join(DIR, 'experiments/raw')
    EXPERIMENTS_TAR_DIR = os.path.join(DIR, 'experiments/tar')
    RESULTS_DIR = os.path.join(DIR, 'results')
    TEMP_DIR = os.path.join(DIR, 'temp')

    # Iterate through each problem set
    for _, dirs, _ in os.walk(EXPERIMENTS_DIR):
        for dname in dirs:
            print("dname")
            domaindir = f"{EXPERIMENTS_DIR}/{dname}/domain.pddl"
            hypsdir = f"{EXPERIMENTS_DIR}/{dname}/hyps.dat"
            realhypdir = f"{EXPERIMENTS_DIR}/{dname}/real_hyp.dat"
            templatedir = f"{EXPERIMENTS_DIR}/{dname}/template.pddl"
            
            extraction = ExtractLandmarks(domaindir, hypsdir, realhypdir, templatedir, debug=True)
            landmarkslst = extraction.reducing()

    # For TAR files
    '''
    for _, _, files in os.walk(EXPERIMENTS_TAR_DIR):
        for tarfname in files:
            extraction = ExtractLandmarks(f"{EXPERIMENTS_TAR_DIR}/{tarfname}")
    '''


