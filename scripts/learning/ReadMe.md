
When creating a member from component populations,
the components are chosen randomly.

How should different scores be stored?
By extension, the current recording method for different terrains should change.

In algorithms, a member of the previous generation is selected for mutation.
This is the same approach as with the previous architecture (Elitism).
However, this new member is directly copied and used as a template for mutations.
Thus, if there was information specific to the previous member, it MUST be overwritten
during the creation/mutation of the new member.

In the current implementation, multiple scores for a single trial have equal weight in a component.
If the same component is used in different learning scenarios, then it may be of value to have "subscores"
for when different terrains / fitness functions are used.
Otherwise, single-run scores get watered down.

EmptyComponent is sticking around to give a template to MonteCarlo when there is no seed.

I would like to change Generation in the following manner:
    - Add a ComponentPopulation subclass which extends list
            - Provides improved usability with dynamically calculating totalScore, sorting, etc...
            - Is a list of Component objects
    - Add a Component class which extends dict
            - Automatically sortable by score, either max or average
    - Generation extends dict with ComponentPopulation elements
    - Both of these factors are unknown to the developer writing Jobs/Algorithms
        - Generation and dict could be used interchangeably
        - Conversion to Generation/ComponentPopulation would be done in the Algorithms phase
    - All classes would have a deepcopy function so that "import copy" isn't needed everywhere

    - Given that this is not a functionality improvement, I have deferred it

Consider making an AlgorithmJob object:
    - Takes the scoreMethod and fitnessFunction values so they don't need to be passed in Algorithms

There is a concurrency read/write error for the .json controller files
    - Easily shows in testGASpec.json with 1< terrains

Currently, rangeConfig is being passed through dictionary recursion when mutating values.
This is in anticipation of property or structure learning needing to select a value from a
list, instead of from a range.
    - Might not be necessary

Currently, __mutateParams is mutating by random.normalvariate(0, config['mutationAmount']
    - The random value is used for BOTH ranges of -1..1 and 0..1
    - This seems to be inconsistent with standard mutation
        - Double the change for 0..1 as for -1..1

TODO:
Exception Hierarchy
Refactor out component name from generatecomponent(s)populationss