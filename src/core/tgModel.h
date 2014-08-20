/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

#ifndef TG_MODEL_H
#define TG_MODEL_H

/**
 * @file tgModel.h
 * @brief Contains the definition of class tgModel.
 * @author Ryan Adams
 * $Id$
 */

// This application
#include "tgCast.h"
#include "tgTaggable.h"
#include "tgTagSearch.h"
#include "abstractMarker.h"
// The C++ Standard Library
#include <iostream>
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgWorld;

// Forward declaration for use with the data logger redesign
class tgModelVisitor_tgDLR;
// end forward declarations for the tgDLR

/**
 * A root-level model is a Tensegrity. It can contain sub-models.
 * The Composite design pattern is used for the sub-models.
 */
class tgModel : public tgTaggable
{
public: 

    /**
    * The default constructor. Primarily used within the tgCreator for
    * compounded objects and subclasses of tgModel
    */  
    tgModel();

    /**
    * Constructor for a model with tags, used within tgCreator for things
    * that are already tagged
    * @param[in] tgTags, tags for the tgTaggable parent class
    */
    tgModel(const tgTags& tags);

    /**
    * Constructor for sace seperated tags
    * @param[in] space_seperated_tags, tags for the tgTaggable parent class
    */  
    tgModel(std::string space_separated_tags);

    /**
    * Destructor. Deletes the children, if they weren't already deleted
    * by teardown()
    */
    virtual ~tgModel();
    
    /**
     * Setup takes a tgWorld and passes it to any children for their
     * own setup functions. All subclasses should call this at the 
     * appropriate time (usually end of setup) within their own
     * setup function.
     * @param[in] world - the tgWorld the models will exist in.
     */
    virtual void setup(tgWorld& world);
    
    /**
     * Deletes the children (undoes setup)
     */
    virtual void teardown();

    /**
    * Advance the simulation.
    * @param[in] dt the number of seconds since the previous call;
    * std::invalid_argument is thrown if dt is not positive
    * @throw std::invalid_argument if dt is not positive
    * @note This is not necessarily const for every child.
    */
    virtual void step(double dt);

    /**
    * Call tgModelVisitor::render() on self and all descendants.
    * @param[in,out] r a reference to a tgModelVisitor
    */
    virtual void onVisit(const tgModelVisitor& r) const;

    /**
    * Add a sub-model to this model.
    * The model takes ownership of the child sub-model and is responsible for
    * deallocating it.
    * @param[in,out] pChild a pointer to a sub-model
    * @throw std::invalid_argument is pChild is NULL, this object, or already
    * a descendant
    * @todo Make sure that every child appears no more than once in the tree.
    */
    void addChild(tgModel* pChild);
	
	/**
	 * Returns the tag names of this model and its children
	 * @param[in] prefix a string to append to
	 * @return the original string with this model and its children's
	 * tags appended
	 */
    virtual std::string toString(std::string prefix = "") const;
	
	/**
	 * Get a vector of descendants sorted by type and a tagsearch.
	 * Useful for pulling out muscle groups, or similar.
	 * @param[in] tagSearch, a tagSearch that contains the desired tags
	 * @return a std::vector of pointers to members that match the tag
	 * search and typename T
	 */
    template <typename T>
    std::vector<T*> find(const tgTagSearch& tagSearch)
    {
        return tgCast::find<tgModel, T>(tagSearch, getDescendants());
    }
	
	/**
	 * Get a vector of descendants sorted by type and a tagsearch.
	 * Useful for pulling out muscle groups, or similar.
	 * @param[in] tagSearch, a std::string& that contains the desired tags
	 * @return a std::vector of pointers to members that match the tag
	 * search and typename T
	 */
    template <typename T>
    std::vector<T*> find(const std::string& tagSearch)
    {
        return tgCast::find<tgModel, T>(tgTagSearch(tagSearch), getDescendants());
    }

    /**
     * Return a std::vector of const pointers to all sub-models.
     * @todo examine whether this should be public, and perhaps create
     * a read only version
     * @return a std::vector of const pointers all sub-models.
     */
    std::vector<tgModel*> getDescendants() const;

    const std::vector<abstractMarker>& getMarkers() const {
        return m_markers;
    }

    void addMarker(abstractMarker a){
        m_markers.push_back(a);
    }

    // Below this line are all the tgDataObserver hacks.

    /**
     * Attach an tgModel observer to the subject of the observer.
     * @param[in,out] pObserver a pointer to an observer for the subject;
     * do nothing if the pointer is NULL
     */
    void attachtgModel(tgObserver<tgModel>* pObserver);

    /**
     * Call tgObserver<tgModel>::onStep() on all observers in the 
     * order in which they were attached, on those specific to tgModel.
     * @param[in] dt the number of seconds since the previous call; do nothing
     * if not positive
     */
    void notifySteptgModel(double dt);

    /**
     * Call tgObserver<tgModel>::onSetup() on all observers in the order 
     * in which they were attached, on those specific to tgModel
     */
    void notifySetuptgModel();

    /**
     * Call tgObserver<tgModel>::onTeardown() on all observers in the order 
     * in which they were attached, on those specific to tgModel.
     */
    void notifyTeardowntgModel();

    // remove the above once we make everything actually polymorphic

    // below this line are functions that will not be needed once the data logger
    // redesign code is merged into the main core - for now, if it was changed,
    // others' loggers would break!

    /**
     * Call tgModelVisitor::render() on self and all descendants.
     * @param[in,out] r a reference to a tgModelVisitor
     */
    virtual void onVisit(const tgModelVisitor_tgDLR& r) const;

    // end data logger redesign methods
	

private:

    /** Integrity predicate. */
    bool invariant() const;

private:

    /**
     * The collection of child models.
     * @note This could be an std::set, but std::vector is more convenient for
     * iterating without using an algorithm.
     */
    std::vector<tgModel*> m_children;

    std::vector<abstractMarker> m_markers;

    /**
     * A sequence of observers called in the order in which they were attached.
     * The subject does not own the observers and must not deallocate them.
     * Note that though this looks the same as in tgSubject, it'll be different
     * because of namespaces.
     */
     std::vector<tgObserver<tgModel> * > m_observers;

};

/**
 * Overload operator<<() to handle tgModel
 * @param[in,out] os an ostream
 * @param[in] pair a tgModel
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
std::ostream&
operator<<(std::ostream& os, const tgModel& obj);
#endif
