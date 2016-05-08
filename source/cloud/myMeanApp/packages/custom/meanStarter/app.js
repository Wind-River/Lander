'use strict';

/*
 * Defining the Package
 */
var Module = require('meanio').Module;

var MeanStarter = new Module('meanStarter');

/*
 * All MEAN packages require registration
 * Dependency injection is used to define required modules
 */
MeanStarter.register(function(app, users, system) {

  // Set views path, template engine and default layout
  app.set('views', __dirname + '/server/views');

  app.get('/lander', function (req, res) {
    console.log("I received a Lander request");
  });

  MeanStarter.angularDependencies(['mean.system', 'mean.users']);

  return MeanStarter;
});
