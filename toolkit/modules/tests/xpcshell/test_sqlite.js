/* Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/ */

"use strict";

const {classes: Cc, interfaces: Ci, utils: Cu} = Components;

do_get_profile();

Cu.import("resource://gre/modules/commonjs/promise/core.js");
Cu.import("resource://gre/modules/osfile.jsm");
Cu.import("resource://gre/modules/Sqlite.jsm");
Cu.import("resource://gre/modules/Task.jsm");

// To spin the event loop in test.
Cu.import("resource://services-common/async.js");

function sleep(ms) {
  let deferred = Promise.defer();

  let timer = Cc["@mozilla.org/timer;1"]
                .createInstance(Ci.nsITimer);

  timer.initWithCallback({
    notify: function () {
      deferred.resolve();
    },
  }, ms, timer.TYPE_ONE_SHOT);

  return deferred.promise;
}

function getConnection(dbName, extraOptions={}) {
  let path = dbName + ".sqlite";
  let options = {path: path};
  for (let [k, v] in Iterator(extraOptions)) {
    options[k] = v;
  }

  return Sqlite.openConnection(options);
}

function getDummyDatabase(name, extraOptions={}) {
  const TABLES = {
    dirs: "id INTEGER PRIMARY KEY AUTOINCREMENT, path TEXT",
    files: "id INTEGER PRIMARY KEY AUTOINCREMENT, dir_id INTEGER, path TEXT",
  };

  let c = yield getConnection(name, extraOptions);
  c._initialStatementCount = 0;

  for (let [k, v] in Iterator(TABLES)) {
    yield c.execute("CREATE TABLE " + k + "(" + v + ")");
    c._initialStatementCount++;
  }

  throw new Task.Result(c);
}


function run_test() {
  Cu.import("resource://testing-common/services-common/logging.js");
  initTestLogging("Trace");

  run_next_test();
}

add_task(function test_open_normal() {
  let c = yield Sqlite.openConnection({path: "test_open_normal.sqlite"});
  yield c.close();
});

add_task(function test_open_unshared() {
  let path = OS.Path.join(OS.Constants.Path.profileDir, "test_open_unshared.sqlite");

  let c = yield Sqlite.openConnection({path: path, sharedMemoryCache: false});
  yield c.close();
});

add_task(function test_get_dummy_database() {
  let db = yield getDummyDatabase("get_dummy_database");

  do_check_eq(typeof(db), "object");
  yield db.close();
});

add_task(function test_simple_insert() {
  let c = yield getDummyDatabase("simple_insert");

  let result = yield c.execute("INSERT INTO dirs VALUES (NULL, 'foo')");
  do_check_true(Array.isArray(result));
  do_check_eq(result.length, 0);
  yield c.close();
});

add_task(function test_simple_bound_array() {
  let c = yield getDummyDatabase("simple_bound_array");

  let result = yield c.execute("INSERT INTO dirs VALUES (?, ?)", [1, "foo"]);
  do_check_eq(result.length, 0);
  yield c.close();
});

add_task(function test_simple_bound_object() {
  let c = yield getDummyDatabase("simple_bound_object");
  let result = yield c.execute("INSERT INTO dirs VALUES (:id, :path)",
                               {id: 1, path: "foo"});
  do_check_eq(result.length, 0);
  do_check_eq(c.lastInsertRowID, 1);
  do_check_eq(c.affectedRows, 1);
  yield c.close();
});

// This is mostly a sanity test to ensure simple executions work.
add_task(function test_simple_insert_then_select() {
  let c = yield getDummyDatabase("simple_insert_then_select");

  yield c.execute("INSERT INTO dirs VALUES (NULL, 'foo')");
  yield c.execute("INSERT INTO dirs (path) VALUES (?)", ["bar"]);

  let result = yield c.execute("SELECT * FROM dirs");
  do_check_eq(result.length, 2);

  let i = 0;
  for (let row of result) {
    i++;

    do_check_eq(row.numEntries, 2);
    do_check_eq(row.getResultByIndex(0), i);

    let expected = {1: "foo", 2: "bar"}[i];
    do_check_eq(row.getResultByName("path"), expected);
  }

  yield c.close();
});

add_task(function test_repeat_execution() {
  let c = yield getDummyDatabase("repeat_execution");

  let sql = "INSERT INTO dirs (path) VALUES (:path)";
  yield c.executeCached(sql, {path: "foo"});
  yield c.executeCached(sql);

  let result = yield c.execute("SELECT * FROM dirs");

  do_check_eq(result.length, 2);

  yield c.close();
});

add_task(function test_table_exists() {
  let c = yield getDummyDatabase("table_exists");

  do_check_false(yield c.tableExists("does_not_exist"));
  do_check_true(yield c.tableExists("dirs"));
  do_check_true(yield c.tableExists("files"));

  yield c.close();
});

add_task(function test_index_exists() {
  let c = yield getDummyDatabase("index_exists");

  do_check_false(yield c.indexExists("does_not_exist"));

  yield c.execute("CREATE INDEX my_index ON dirs (path)");
  do_check_true(yield c.indexExists("my_index"));

  yield c.close();
});

add_task(function test_close_cached() {
  let c = yield getDummyDatabase("close_cached");

  yield c.executeCached("SELECT * FROM dirs");
  yield c.executeCached("SELECT * FROM files");

  yield c.close();
});

add_task(function test_execute_invalid_statement() {
  let c = yield getDummyDatabase("invalid_statement");

  let deferred = Promise.defer();

  do_check_eq(c._anonymousStatements.size, 0);

  c.execute("SELECT invalid FROM unknown").then(do_throw, function onError(error) {
    deferred.resolve();
  });

  yield deferred.promise;

  // Ensure we don't leak the statement instance.
  do_check_eq(c._anonymousStatements.size, 0);

  yield c.close();
});

add_task(function test_on_row_exception_ignored() {
  let c = yield getDummyDatabase("on_row_exception_ignored");

  let sql = "INSERT INTO dirs (path) VALUES (?)";
  for (let i = 0; i < 10; i++) {
    yield c.executeCached(sql, ["dir" + i]);
  }

  let i = 0;
  yield c.execute("SELECT * FROM DIRS", null, function onRow(row) {
    i++;

    throw new Error("Some silly error.");
  });

  do_check_eq(i, 10);

  yield c.close();
});

// Ensure StopIteration during onRow causes processing to stop.
add_task(function test_on_row_stop_iteration() {
  let c = yield getDummyDatabase("on_row_stop_iteration");

  let sql = "INSERT INTO dirs (path) VALUES (?)";
  for (let i = 0; i < 10; i++) {
    yield c.executeCached(sql, ["dir" + i]);
  }

  let i = 0;
  let result = yield c.execute("SELECT * FROM dirs", null, function onRow(row) {
    i++;

    if (i == 5) {
      throw StopIteration;
    }
  });

  do_check_null(result);
  do_check_eq(i, 5);

  yield c.close();
});

add_task(function test_invalid_transaction_type() {
  let c = yield getDummyDatabase("invalid_transaction_type");

  let errored = false;
  try {
    c.executeTransaction(function () {}, "foobar");
  } catch (ex) {
    errored = true;
    do_check_true(ex.message.startsWith("Unknown transaction type"));
  } finally {
    do_check_true(errored);
  }

  yield c.close();
});

add_task(function test_execute_transaction_success() {
  let c = yield getDummyDatabase("execute_transaction_success");

  do_check_false(c.transactionInProgress);

  yield c.executeTransaction(function transaction(conn) {
    do_check_eq(c, conn);
    do_check_true(conn.transactionInProgress);

    yield conn.execute("INSERT INTO dirs (path) VALUES ('foo')");
  });

  do_check_false(c.transactionInProgress);
  let rows = yield c.execute("SELECT * FROM dirs");
  do_check_true(Array.isArray(rows));
  do_check_eq(rows.length, 1);

  yield c.close();
});

add_task(function test_execute_transaction_rollback() {
  let c = yield getDummyDatabase("execute_transaction_rollback");

  let deferred = Promise.defer();

  c.executeTransaction(function transaction(conn) {
    yield conn.execute("INSERT INTO dirs (path) VALUES ('foo')");
    print("Expecting error with next statement.");
    yield conn.execute("INSERT INTO invalid VALUES ('foo')");

    // We should never get here.
    do_throw();
  }).then(do_throw, function onError(error) {
    deferred.resolve();
  });

  yield deferred.promise;

  let rows = yield c.execute("SELECT * FROM dirs");
  do_check_eq(rows.length, 0);

  yield c.close();
});

add_task(function test_close_during_transaction() {
  let c = yield getDummyDatabase("close_during_transaction");

  yield c.execute("INSERT INTO dirs (path) VALUES ('foo')");

  let errored = false;
  try {
    yield c.executeTransaction(function transaction(conn) {
      yield c.execute("INSERT INTO dirs (path) VALUES ('bar')");
      yield c.close();
    });
  } catch (ex) {
    errored = true;
    do_check_eq(ex.message, "Connection being closed.");
  } finally {
    do_check_true(errored);
  }

  let c2 = yield getConnection("close_during_transaction");
  let rows = yield c2.execute("SELECT * FROM dirs");
  do_check_eq(rows.length, 1);

  yield c2.close();
});

add_task(function test_detect_multiple_transactions() {
  let c = yield getDummyDatabase("detect_multiple_transactions");

  yield c.executeTransaction(function main() {
    yield c.execute("INSERT INTO dirs (path) VALUES ('foo')");

    let errored = false;
    try {
      yield c.executeTransaction(function child() {
        yield c.execute("INSERT INTO dirs (path) VALUES ('bar')");
      });
    } catch (ex) {
      errored = true;
      do_check_true(ex.message.startsWith("A transaction is already active."));
    } finally {
      do_check_true(errored);
    }
  });

  let rows = yield c.execute("SELECT * FROM dirs");
  do_check_eq(rows.length, 1);

  yield c.close();
});

add_task(function test_shrink_memory() {
  let c = yield getDummyDatabase("shrink_memory");

  // It's just a simple sanity test. We have no way of measuring whether this
  // actually does anything.

  yield c.shrinkMemory();
  yield c.close();
});

add_task(function test_no_shrink_on_init() {
  let c = yield getConnection("no_shrink_on_init",
                              {shrinkMemoryOnConnectionIdleMS: 200});

  let oldShrink = c.shrinkMemory;
  let count = 0;
  Object.defineProperty(c, "shrinkMemory", {
    value: function () {
      count++;
    },
  });

  // We should not shrink until a statement has been executed.
  yield sleep(220);
  do_check_eq(count, 0);

  yield c.execute("SELECT 1");
  yield sleep(220);
  do_check_eq(count, 1);

  yield c.close();
});

add_task(function test_idle_shrink_fires() {
  let c = yield getDummyDatabase("idle_shrink_fires",
                                 {shrinkMemoryOnConnectionIdleMS: 200});
  c._clearIdleShrinkTimer();

  let oldShrink = c.shrinkMemory;
  let shrinkPromises = [];

  let count = 0;
  Object.defineProperty(c, "shrinkMemory", {
    value: function () {
      count++;
      let promise = oldShrink.call(c);
      shrinkPromises.push(promise);
      return promise;
    },
  });

  // We reset the idle shrink timer after monkeypatching because otherwise the
  // installed timer callback will reference the non-monkeypatched function.
  c._startIdleShrinkTimer();

  yield sleep(220);
  do_check_eq(count, 1);
  do_check_eq(shrinkPromises.length, 1);
  yield shrinkPromises[0];
  shrinkPromises.shift();

  // We shouldn't shrink again unless a statement was executed.
  yield sleep(300);
  do_check_eq(count, 1);

  yield c.execute("SELECT 1");
  yield sleep(300);

  do_check_eq(count, 2);
  do_check_eq(shrinkPromises.length, 1);
  yield shrinkPromises[0];

  yield c.close();
});

add_task(function test_idle_shrink_reset_on_operation() {
  const INTERVAL = 500;
  let c = yield getDummyDatabase("idle_shrink_reset_on_operation",
                                 {shrinkMemoryOnConnectionIdleMS: INTERVAL});

  c._clearIdleShrinkTimer();

  let oldShrink = c.shrinkMemory;
  let shrinkPromises = [];
  let count = 0;

  Object.defineProperty(c, "shrinkMemory", {
    value: function () {
      count++;
      let promise = oldShrink.call(c);
      shrinkPromises.push(promise);
      return promise;
    },
  });

  let now = new Date();
  c._startIdleShrinkTimer();

  let initialIdle = new Date(now.getTime() + INTERVAL);

  // Perform database operations until initial scheduled time has been passed.
  let i = 0;
  while (new Date() < initialIdle) {
    yield c.execute("INSERT INTO dirs (path) VALUES (?)", ["" + i]);
    i++;
  }

  do_check_true(i > 0);

  // We should not have performed an idle while doing operations.
  do_check_eq(count, 0);

  // Wait for idle timer.
  yield sleep(INTERVAL);

  // Ensure we fired.
  do_check_eq(count, 1);
  do_check_eq(shrinkPromises.length, 1);
  yield shrinkPromises[0];

  yield c.close();
});

add_task(function test_in_progress_counts() {
  let c = yield getDummyDatabase("in_progress_counts");
  do_check_eq(c._statementCounter, c._initialStatementCount);
  do_check_eq(c.inProgress(), 0);
  yield c.executeCached("INSERT INTO dirs (path) VALUES ('foo')");
  do_check_eq(c._statementCounter, c._initialStatementCount + 1);
  do_check_eq(c.inProgress(), 0);

  let expectOne;
  let expectTwo;

  // Please forgive me.
  let inner = Async.makeSpinningCallback();
  let outer = Async.makeSpinningCallback();

  // We want to make sure that two queries executing simultaneously
  // result in `inProgress()` reaching 2, then dropping back to 0.
  //
  // To do so, we kick off a second statement within the row handler
  // of the first, then wait for both to finish.

  yield c.executeCached("SELECT * from dirs", null, function onRow() {
    // In the onRow handler, we're still an outstanding query.
    // Expect a single in-progress entry.
    expectOne = c.inProgress();

    // Start another query, checking that after its statement has been created
    // there are two statements in progress.
    let p = c.executeCached("SELECT 10, path from dirs");
    expectTwo = c.inProgress();

    // Now wait for it to be done before we return from the row handler …
    p.then(function onInner() {
      inner();
    });
  }).then(function onOuter() {
    // … and wait for the inner to be done before we finish …
    inner.wait();
    outer();
  });

  // … and wait for both queries to have finished before we go on and 
  // test postconditions.
  outer.wait();

  do_check_eq(expectOne, 1);
  do_check_eq(expectTwo, 2);
  do_check_eq(c._statementCounter, c._initialStatementCount + 3);
  do_check_eq(c.inProgress(), 0);

  yield c.close();
});

