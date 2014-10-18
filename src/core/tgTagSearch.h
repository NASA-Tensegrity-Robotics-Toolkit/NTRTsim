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

/**
 * @file tgTagSearch.h
 * @brief Contains the definition of class tgTagSearch
 * @author Ryan Adams
 * $Id$
 */

#ifndef TG_TAG_SEARCH_H
#define TG_TAG_SEARCH_H

#include <string>

#include "tgTags.h"
#include "tgTaggable.h"

/*  @todo: Implement more advanced search capability

    Currently, tag searches just matches all tags, so something like
    tgTagSearch("a b").matches(tgTags("a b c")) is true. 

    Plans: search operators like 'or' and 'not', for instance
    - tgTagSearch("a -b") would match tgTags("a c") but not tgTags("a b")
    - tgTagSearch("a b|c") would match tgTags("a b") and tgTags("a c") 
      but not tgTags("a d")
*/

/**
 * Represents a search to be performed on a tgTaggable
 */
class tgTagSearch
{
public:
    
    tgTagSearch() {}

    tgTagSearch(std::string search_string) : m_search(search_string)
    {}
    
    virtual ~tgTagSearch() {}

    /**
     * Do the tags match this search?
     */
    const bool matches(const tgTags& tags) const
    {
        // Simple for now, just check that the tags contain the tags in the 
        // search
        return tags.contains(m_search);
    }

    const bool matches(const tgTaggable& taggable) const
    {
        return matches(taggable.getTags());
    }

    /**
     * Allows matching of children with the parent's tags virtually added to 
     * all children that are being searched
     */
    bool matches(const tgTags& parentTags, const tgTags& tags)
    {
        // @todo: make sure this works correctly...
        tgTags s(parentTags);
        s.append(tags);
        return matches(s);
    }
    
    /**
     * Remove the given tags from the search
     */
    void remove(const tgTags& tags)
    {
        tgTags s(tags);
        m_search.remove(tags);
    }
    
private:
    
    // @todo: change this to a parsed representation of the and/or/not setup
    tgTags m_search;

};


#endif
