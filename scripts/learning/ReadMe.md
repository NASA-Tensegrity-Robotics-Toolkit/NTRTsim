When creating a member from component populations,
the components are chosen randomly.

In algorithms, a member of the previous generation is selected for mutation.
This is the same approach as with the previous architecture (Elitism).
However, this new member is directly copied and used as a template for mutations.
Thus, if there was information specific to the previous member, it MUST be overwritten
during the creation/mutation of the new member.

In the current implementation, multiple scores for a single trial have equal weight in a component.
If the same component is used in different learning scenarios, then it may be of value to have "subscores"
for when different terrains / fitness functions are used.
Otherwise, single-run scores get watered down.

Removed template component for all implementations. You must have a seed file.
This requirement for the ControllerJobMaster class could be introduced with interface inheritance.

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

There is a concurrency read/write error for the .json controller files
    - Easily shows in testGASpec.json with 1< terrains
    - Likely only a problem for testGASpec, since it runs so fast

Currently, mutateParams is mutating by random.normalvariate(0, config['mutationAmount']
    - The random value is used for BOTH ranges of -1..1 and 0..1
    - This seems to be inconsistent with standard mutation
        - Double the change for 0..1 as for -1..1
    - This approach is NOT used in the new learning

The dictionary walkers used for the learning algorithms are all very similar.
    - Would be nice to find a way to use one walker and call it for each alg
    There are two ways to do this:
        1. Implement a "tagStack"
            1.1 mutateDictionary(dictionary, mutatorFunction): 
            1.2 mutatorFunction, specified by the developer, is applied to every leaf node in the dictionary
            1.3 At each leaf node, two values are available:
                1.3.1 tagStack = list of parent nodes leading to the leaf node
                1.3.2 element  = the value at the leaf node
            1.4 For Algorithms, would require a closure/partial function where tagStack is compared to rangeConfig
                1.4.1 The enclosing function takes in algConfig, rangeConfig, returned function takes tagStack, element 
            1.5 O(nlogn) due to n elements and tagStack of size logn
            + More generic. Essentially just matching tags to leaf nodes
            - Developer needs to know closure/partial function implementation in Python
        2. Walk the dictionary directly:
            2.1 mutateDictionary(dictionary, algConfig, rangeConfig, mutatorFunction):
            2.2 rangeConfig is walked simultaneously with dictionary (recursive implementation)
            2.3 at a leaf node, only the range/minMax of that element is present
            2.4 mutatorFunction still applies to every leaf node
                2.4.1 mutatorFunction(element, range, algConfig)
            - Less Generic. Would require algConfig, unlike 1.
            + Simpler for a new developer to implement
        In either case, the passed-in dictionary is MUTATED
            Could possibly avoid this by doing a copy operation at each level (expensive)
        For now, a direct implementation of the walker for each algorithm is used
            This may be refactored to follow a single walker, such as 1, or 2, after property learning is implemented
            
    - An even better option would be to be able to both mutate the target value and get a separate return value
        - i.e. fold over all elements
        - This would be possible by "flattening" the dictionary, or maybe with the dictwalker+tagStack
        
Brian's implementation of crossOver may have a flaw:
    - cNew[crossOver:len(c2)+1] = c2[crossOver:len(c2)+1]
    - This code will not produce a different child for a 1-D array, such as in params below
    - Thus, no real crossover is being applied to this component
    - This param configuration is due to the NN configuration (double check this)
    - For general learning, unknown array configurations must be supported
        Exable case:
		"params" : 
		[
			[
				0.20951663264338694,
				0.066759208171998541,
				0.81064210055409802,
				0.78043146109390282,
				0.60541811415525382
			]
		],    


The overall process of creating a new component from an old component is as follows:
    1. Copy the old component
    2. Walk through it, matching its elements with those of rangeConfig
        2.1 At each matched element, mutate it according to the algorithm
        
The Ranges section of a learningSpec must follow this format:
    A tag is a dictionary.
    If a tag contains another tag, then it can only contain tags.
    If a tag has the two elements min: number max: number, it is a terminal.
    If a tag has a list of elements, it is a terminal.
    Example:
        tag:
            tag:
                min: number
                max: number
            tag:
                - option1
                - option2
                ...

If a value is specified in the learning range config but not in the component, it's an error.
This requires all parameters that want to be learned over to be pre-defined in the yaml file.
    - May reduce usability for large descriptions

TODO:
Exception Hierarchy
Refactor out component name from generatecomponent(s)populationss
Update scores after beginTrial
During the refactoring pass, check each use of copy.deepcopy(x). Likely overusing it