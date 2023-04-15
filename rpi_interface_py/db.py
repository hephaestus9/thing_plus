# -*- coding: utf-8 -*-
# Copyright (c) 2006-2010, Jesse Liesch
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE IMPLIED
# DISCLAIMED. IN NO EVENT SHALL JESSE LIESCH BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import threading
import sqlite3 as sqlite


class Db:
    def __init__(self, name, host="", user="", password=""):
        self.name = name
        self.host = host
        self.user = user
        self.password = password

        self.conns = {}
        self.connParams = {}
        self.transactionDepth = 0
        self.lastQuery = False

    def close(self):
        self.getConn().close()

    def getConnParam(self):
        threadId = threading.currentThread().getName()
        # Make sure we have a connection
        if not threadId in self.conns:
            self.getConn()
        return self.connParams[threadId]

    def getConn(self):
        def dict_factory(cursor, row):
            d = {}
            for idx, col in enumerate(cursor.description):
                d[col[0]] = row[idx]
            return d

        def boolAdapter(b):
            if b:
                return 'True'
            else:
                return 'False'

        # Return connection for current thread
        threadId = threading.currentThread().getName()
        if not threadId in self.conns:
            sqlite.register_adapter(bool, boolAdapter)
            conn = sqlite.connect(self.name, timeout=30)
            conn.isolation_level = None
            conn.row_factory = dict_factory
            self.connParams[threadId] = "?"
            self.conns[threadId] = conn
        return self.conns[threadId]

    def checkTable(self, name, fields, index=[], unique=[]):
        # Only allow one thread to check a table at a time
        # Otherwise we may have two threads create/update the same table

        # First create empty table
        try:
            createString = "create table if not exists " + name + "("
            first = True
            for f in fields:
                if first:
                    first = False
                else:
                    createString += ", "
                createString += f["name"] + " " + f["type"]
            createString += ")"
            self.getConn().execute(createString)
        except Exception as e:
            print(e)
            pass

        meta = self.getConn().execute('select * from ' + name).description

        # Check for new fields
        for i in range(len(fields)):
            f = fields[i]
            found = False
            for m in meta:
                if f["name"] == m[0]:
                    found = True
                    break
            if not found:
                # Create field
                alterString = "alter table " + name + " add column " + f["name"] + " " + f["type"]
                self.getConn().execute(alterString)

        # Build index
        for i in index:
            s = "create index if not exists " + i["name"] + " on " + name + "("
            first = True
            for col in i["cols"]:
                if first:
                    first = False
                else:
                    s += ", "

                s += col
            s += ")"

            self.getConn().execute(s)

        # Build unique index
        for i in unique:
            s = "create unique index if not exists " + i["name"] + " on " + name + "("
            first = True
            for col in i["cols"]:
                if first:
                    first = False
                else:
                    s += ", "

                s += col
            s += ")"

            self.getConn().execute(s)

    def query(self, queryStr, tuple=False, reRaiseException=False):
        reRaiseException = True
        try:
            if tuple:
                self.lastQuery = "%s %s" % (queryStr, tuple)
                #print self.lastQuery
                return self.getConn().execute(queryStr, tuple)
            else:
                self.lastQuery = queryStr
                return self.getConn().execute(queryStr)
        except Exception as e:
            print(e)
            if reRaiseException:
                raise

            # Return empty string
            return self.getConn().execute("select 0 where 1 = 0")

    def delete(self, table, where=False):
        deleteStr = "delete from " + table
        deleteTuple = []
        if where:
            deleteStr += " where "
            first = True
            for key in where:
                if first:
                    first = False
                else:
                    deleteStr += " and "

                deleteStr += key + "=" + self.getConnParam()
                deleteTuple.append(where[key])

        self.query(deleteStr, deleteTuple)

    def select(self, table, orderBy=False, where=False, limit=False, what=False):
        selectStr = "select "
        if what:
            selectStr += what
        else:
            selectStr += "*"
        selectStr += " from " + table
        selectTuple = []

        # TODO: make sure key is not bad
        if where:
            selectStr += " where "
            first = True
            for key in list(where.keys()):
                if first:
                    first = False
                else:
                    selectStr += " and "
                if where[key] == "is null" or where[key] == "is not null":
                    selectStr += key + " " + where[key]
                else:
                    selectStr += key
                    if key.find("=") == -1 and key.find(">") == -1 and key.find("<") == -1:
                        selectStr += "=" + self.getConnParam()
                    else:
                        selectStr += "" + self.getConnParam()
                    selectTuple.append(where[key])

        if orderBy:
            selectStr += " order by " + orderBy

        if limit:
            selectStr += " limit " + str(limit)

        return self.query(selectStr, selectTuple)

    def insert(self, table, data):
        insertStr = "insert into " + table + " ("
        insertTuple = []

        first = True
        for key in list(data.keys()):
            if first:
                first = False
            else:
                insertStr += ", "
            insertStr += key

        insertStr += ") values ("

        first = True
        for i in list(data.keys()):
            if first:
                first = False
                insertStr += self.getConnParam()
            else:
                insertStr += ", " + self.getConnParam()
            insertTuple.append(data[i])

        insertStr += ")"
        #print insertStr, insertTuple 
        return self.query(insertStr, insertTuple)

    def update(self, table, data, where):
        updateStr = "update " + table + " set "
        updateTuple = []

        # TODO: make sure key is not bad
        first = True
        for key in list(data.keys()):
            if first:
                first = False
            else:
                updateStr += ", "
            updateStr += key + "=" + self.getConnParam()
            updateTuple.append(data[key])

        updateStr += " where "

        first = True
        for key in list(where.keys()):
            if first:
                first = False
            else:
                updateStr += " and "
            if where[key] == "is null" or where[key] == "is not null":
                updateStr += key + " " + where[key]
            else:
                updateStr += key + "=" + self.getConnParam()
                updateTuple.append(where[key])

        return self.query(updateStr, updateTuple)

    # Return true on insert, false on update
    def insertOrUpdate(self, table, data, on={}):
        if not on:
            # If on is empty insert if data is not find
            result = self.select(table, where=data)
            if not result.fetchone():
                self.insert(table, data)
                return True
        else:
            # Select by on.  If found and different, update.
            # If found and the same do nothing
            # If not found insert.
            result = self.select(table, where=on)
            # TODO: handle update
            if not result.fetchone():
                self.insert(table, data)
                return True
            else:
                self.update(table, data, on)
        return False

    def inTransaction(self):
        return self.transactionDepth > 0

    def beginTransaction(self):
        self.transactionDepth += 1
        if self.transactionDepth == 1:
            self.getConn().execute("begin immediate transaction")
        #if self.transactionDepth == 1:
        #    print "DB begin transaction"

    def rollbackTransaction(self):
        self.transactionDepth = 0
        try:
            self.getConn().execute("rollback transaction")
        except Exception as e:
            print((e))
        #print "DB rollback transaction"

    def commitTransaction(self):
        if self.transactionDepth == 1:
            self.transactionDepth = 0
            self.getConn().execute("commit transaction")
        if self.transactionDepth >= 1:
            self.transactionDepth -= 1
            #if self.transactionDepth == 0:
            #    print "DB committed transaction"
            #else:
            #    print "DB reduce transaction depth"