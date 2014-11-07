/*
 * Copyright (C) 2013  CoDyCo Consortium
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
 * Authors: Silvio Traversaro, Andrea Del Prete
 * email: silvio.traversaro@iit.it - andrea.delprete@iit.it
 */

#ifndef WBI_ID_H
#define WBI_ID_H

#include <vector>
#include <string>

namespace wbi
{
    
    /**
     * Identifier used by wholeBodyInterface. It is the class used to identify uniquely
     * joints, sensors, links and every other kind of entity in wholeBodyInterface .
     *
     */
    class wbiId
    {
    private:
        std::string id;
    public:
        wbiId();
        
        wbiId(const std::string & new_id);
        
        wbiId(const char * new_id);
        
        
        wbiId & operator=(const wbiId & id_copy);
        
        /**
         * Get the identifier string for the wbiId
         */
        const std::string & toString() const;
        
        bool operator==(const wbiId & comparison_id) const;
    };
    
    /**
     * List of wholeBodyInterface identifiers (wbiId).
     * Used to represent lists of joints, sensors, links, ... .
     */
    class wbiIdList
    {
    private:
        std::vector<wbiId> storage;
        
    protected:
        /** Add the specified id without checking its existance. */
        void pushId(const wbiId & id);
        
    public:
        wbiIdList();
        wbiIdList(const wbiId &id0);
        /**
         * Create an id list with the specified ids.
         * @param id0 First id
         * @param id1 Second id
         */
        
        /** \note this helper function should be deprecated
         *        as they facilitate putting wbiIdList in code
         *        while they should be in configuration files.
         */
        wbiIdList(const wbiId &id0, const wbiId &id1);
        wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2);
        wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3);
        wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3, const wbiId &id4);
        wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3, const wbiId &id4, const wbiId& id5);
        wbiIdList(const wbiId &id0, const wbiId &id1, const wbiId &id2, const wbiId &id3, const wbiId &id4, const wbiId& id5, const wbiId & id6);
        
        wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2);
        wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3);
        wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3, const wbiIdList &lid4);
        wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3, const wbiIdList &lid4, const wbiIdList &lid5);
        wbiIdList(const wbiIdList &lid1, const wbiIdList &lid2, const wbiIdList &lid3, const wbiIdList &lid4, const wbiIdList &lid5, const wbiIdList &lid6);
        
        virtual ~wbiIdList();
        
        /**
         * Convert a wbiId to a numeric id.
         * Return the numeric id (index) of the specified wbiId in this wbiIdList.
         * @return true it the specified wbiId is found in the list, false otherwise.
         */
        virtual bool wbiIdToNumericId(const wbiId &wbi_id, int & numericId) const;
        
        /**
         * Convert a numeric id to a wbiId
         * @return true it the specified wbiId is found in the list, false otherwise
         */
        virtual bool numericIdToWbiId(const int numeridId, wbiId & wbi_id) const;
        
        /**
         * Remove the specified id from the list
         * @return true if the id was found and removed, false if it was not found
         */
        virtual bool removeId(const wbiId &id);
        
        /**
         * Add the specified id to the list.
         * @param id id to add
         * @return true if the id has been added
         */
        virtual bool addId(const wbiId &id);
        
        /**
         * Add the specified ids to the list.
         *
         * \note this function has a computational complexity of O(n^2),
         *       where n is the maximum between the size of the two used lists.
         *
         * @param appendedList id list to add
         * @return the number of ids added to the list
         *        (0 if all the elements in the appendedList were already in the list,
         *         appendedList.size() if all the element in the appendedList were not in the list.
         *
         */
        virtual int addIdList(const wbiIdList &appendedList);
        
        /**
         * Get the number of ids in this list
         * @return the number of ids in the list
         */
        virtual unsigned int size() const;
        
        /**
         * Check whether the specified id is present in this list.
         * @return true if the specified id is in the list, false otherwise.
         */
        virtual bool containsId(const wbiId &i) const;
        
        /**
         * Provide a human readable represent of the list
         */
        virtual std::string toString() const;
    };
    
    
} // end namespace

#endif

