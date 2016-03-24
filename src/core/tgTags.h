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
 * @file tgTags.h
 * @brief Contains the definition of class tgTags
 * @author Ryan Adams
 * $Id$
 */

#ifndef TG_TAGS_H
#define TG_TAGS_H

#include <deque>
#include <set>
#include <string>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <locale>         // std::locale, std::isalnum

#include "tgException.h"

struct tgTagException : public tgException
{
   tgTagException(std::string ss) : tgException(ss) {}
};

class tgTags
{
public:
    tgTags() {}
    tgTags(const std::string& space_separated_tags)
    {
        append(space_separated_tags);
    }
    
    bool contains(const std::string& space_separated_tags) const
    {
        const std::deque<std::string> tags = splitTags(space_separated_tags);
        return contains(tags);
    }

    bool contains(const tgTags& tags) const
    {
        return contains(tags.getTags());
    }
        
    bool containsAny(const std::string& space_separated_tags)
    {
        std::deque<std::string> tags = splitTags(space_separated_tags);
        return containsAny(tags);
    }

    bool containsAny(const tgTags& tags) 
    {
        return containsAny(tags.getTags());
    }

    void append(const std::string& space_separated_tags)
    {
        append(splitTags(space_separated_tags));
    }
    
    void append(const tgTags& tags) 
    {
        append(tags.getTags());
    }
    
    void prepend(const std::string& space_separated_tags)
    {
        prepend(splitTags(space_separated_tags));
    }
    
    void prepend(const tgTags& tags)
    {
        prepend(tags.getTags());
    }
    
    void remove(const std::string& space_separated_tags)
    {
        remove(splitTags(space_separated_tags));
    }

    void remove(const tgTags& tags)
    {
        remove(tags.getTags());
    }

    const int size() const
    {
        return m_tags.size();
    }
    
    const bool empty() const
    {
        return m_tags.empty();
    }

    static std::deque<std::string> splitTags(const std::string &s, char delim = ' ') {
        std::deque<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim)) {
            if(!item.empty())
                elems.push_back(item);
        }
        return elems;
    }
    
    /**
     * Split tags in a set (some of the strings in the set may contain multiple tags)
     * e.g. {'sometag', 'another andanother'} => {'sometag', 'another', 'andanother'}
     */ 
    static std::deque<std::string> splitTags(const std::deque<std::string> &s,
                          char delim = ' ') {
        std::deque<std::string> result;
        for(std::size_t i = 0; i < s.size(); i++) {
            std::deque<std::string> spl = splitTags(s[i]);
            for(std::size_t j = 0; j < spl.size(); j++) {
                result.push_back(spl[j]);
            }
        }
        return result;
    }

    std::string joinTags(std::string delim = "_") {
        std::stringstream ss;
        for(std::size_t i = 0; i < m_tags.size(); i++) {
            if(i != 0) {
                ss << delim;
            }
            ss << m_tags[i];
        }
        return ss.str();
    }

    /**
     * Determine if the string can be cast to an integer
     */
    bool isIntegery(const std::string s) const
    {
        if(s.empty()) {
            return false;
        }
        std::stringstream ss; 
        ss << atoi(s.c_str()); //add number to the stream        
        // If our initial string equals the string converted to int and back to string, it's integery.
        if(ss.str() == s)  
            return true;
        return false;
    }
    
    bool isValid(std::string tag)
    {
        if (tag.empty()) 
            return false;
        
        // Can't be an integer
        if(isIntegery(tag)) {
            return false;
        }
        // Can't contain the delimiter (space)
        if(tag.find(' ') != std::string::npos) {
            return false;
        }
        // Must be alphanumeric, or at least can't start with strange characters or contain things like '|'
        // @todo: Add this functionality. Apparently it's not as straightforward as one might hope...
        //if(!std::isalnum(tag)) {
        //    return false;
        //}
        return true;
    }

    std::deque<std::string>& getTags()
    {
        return m_tags;
    }

    const std::deque<std::string>& getTags() const
    {
        return m_tags;
    }

    /**
     * Return the tags as an unordered set
     */
    const std::set<std::string> asSet() const
    {
        return std::set<std::string>(m_tags.begin(), m_tags.end());
    }

    /**
     * Return a non-const reference to the tag that is indexed by the
     * int key. It must be in m_tags.
     * @param[in] key the key of the tag to retrieve
     * @reeturn a const reference to the tag that is indexed by key
     */
    std::string& operator[](int key) { 
        return m_tags[key]; 
    }
    
    const std::string& operator[](int key) const { 
        return m_tags[key]; 
    }
    
    /**
     * Check if we contain the same tags regardless of ordering
     */
    bool operator==(const tgTags& rhs)
    {
        return rhs.asSet() == asSet(); 
    }

    tgTags& operator+=(const tgTags& rhs)
    {
        const std::deque<std::string>& other = rhs.getTags();
        m_tags.insert(m_tags.end(), other.begin(), other.end());
        return *this;
    }

private:
        
    /**
     * Add a tag that is known to be valid (e.g. doesn't contain illegal chars,
     * isn't an integer, etc.)
     */
    void appendOne(std::string tag) {
        if(!isValid(tag)) {
            throw tgTagException("Invalid tag '" + tag + "' - tags must be alphanumeric and may not be castable to int.");
        }
        if(!containsOne(tag)) {
            m_tags.push_back(tag);
        }
    }
    
    void append(const std::deque<std::string>& tags)
    {
        for(std::size_t i = 0; i < tags.size(); i++) {
            appendOne(tags[i]);
        }
    }
    
    void prependOne(std::string tag) {
        if(isValid(tag) && !containsOne(tag)) {
            m_tags.push_front(tag);
        }
    }

    void prepend(const std::deque<std::string>& tags)
    {
        for(std::size_t i = 0; i < tags.size(); i++) {
            prependOne(tags[i]);
        }
    }

    bool contains(const std::deque<std::string>& tags) const {
        for(std::size_t i = 0; i < tags.size(); i++) {
            if(!containsOne(tags[i])) 
                return false;
        }
        return true;
    }

    bool containsAny(const std::deque<std::string>& tags) const {
        for(std::size_t i = 0; i < tags.size(); i++) {
            if(containsOne(tags[i])) 
                return true;
        }
        return false;
    }
    
    /**
     * Check whether we contain a tag that is known to be valid
     */
    bool containsOne(std::string tag) const {
        return (std::find(m_tags.begin(), m_tags.end(), tag) != m_tags.end());
    }
    
    void removeOne(std::string tag) {
        m_tags.erase(std::remove(m_tags.begin(), m_tags.end(), tag), m_tags.end());
    }
    
    void remove(std::deque<std::string> tags) {
        for(std::size_t i = 0; i < tags.size(); i++) {
            removeOne(tags[i]);
        }
    }
    
    std::deque<std::string> m_tags;
};

/**
 * Overload operator<<() to handle a tgTags.
 * @param[in,out] os an ostream
 * @param[in] cs a tgTags
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
inline std::ostream&
operator<<(std::ostream& os, const tgTags& tags)
{
    const std::deque<std::string>& t = tags.getTags();
    for(size_t i = 0; i < t.size(); ++i)
    {
      if(i != 0)
        os << " ";
        os << t[i];
    }
    return os;
}

inline tgTags operator+(tgTags lhs, const tgTags& rhs)
{
  lhs += rhs;
  return lhs;
}

/**
 * Represent tags as a YAML list
 * Note: this function has no dependencies on external libraries
 */
inline std::string asYamlList(const tgTags& tags)
{
    std::stringstream os;
    os << "[";
    for(size_t i = 0; i < tags.size(); i++) {
        os << '"' << tags[i] << '"';
        if(i != tags.size() - 1) 
            os << ", ";
    }    
    os << "]";
    return os.str();
};

inline bool operator< (const tgTags& lhs, const tgTags& rhs){ return lhs.asSet() < rhs.asSet(); }
inline bool operator> (const tgTags& lhs, const tgTags& rhs){return rhs < lhs;}
inline bool operator<=(const tgTags& lhs, const tgTags& rhs){return !(lhs > rhs);}
inline bool operator>=(const tgTags& lhs, const tgTags& rhs){return !(lhs < rhs);}


#endif
