## Extracting landmarks using Pyperplan
- Basic implementation of intersections between landmarks of different goals in a pandas dataframe.
- Can iterate through different planning problems and extract landmarks.
- Toy problem in experiments/raw/Problem1
- Final framework in Landmark Planning/pyperplan-master/landmarkextraction.py

### To do:
- Store the data somewhere.
- Implement the idea of a candidate goal/real goal.
- Test on some larger datasets.
- Reorganisize folder structure and reliances so that code just needs pyperplan imported rather than directly extending from pyperplan codebase. 
### Issues:
- The PDDL parser does not seem to work with domains which use equality (=).

Domain file is from [Pyperplan's logisitics benchmark](https://github.com/aibasel/pyperplan/blob/master/benchmarks/logistics/domain.pddl).
