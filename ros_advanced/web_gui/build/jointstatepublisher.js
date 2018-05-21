/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

var JOINTSTATEPUBLISHER = JOINTSTATEPUBLISHER || {
  REVISION : '0.0.1-JOINTSTATEPUBLISHER'
};



/**
 * @author David V. Lu!! - davidvlu@gmail.com
 */

/** Replicating the joint_state_publisher node's functionality in the browser 
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * paramName - the parameter to read the robot description from
 *   * topicName - topic to publish joint states on
 *   * divID - the ID of the div to place the sliders
 *   * 
 */
JOINTSTATEPUBLISHER.JointStatePublisher = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var paramName = options.paramName || 'robot_description';
  var topicName = options.topicName || '/web_joint_states';
  
  var sliders = [];
  
  var param = new ROSLIB.Param({
    ros : ros,
    name : paramName
  });
  
  var updateInput = function(event) {
    var name = event.target.name;
    var target;
    if( name.indexOf('_text') >= 0) {
        target = name.replace('_text', '');
    }else{
        target = name + '_text';
    }
    document.getElementById(target).value = event.target.value;
    publish();
  };
   
  var load_model = function(param) {
    var parser = new DOMParser();
    var xmlDoc = parser.parseFromString(param, 'text/xml');
    
    // See https://developer.mozilla.org/docs/XPathResult#Constants
    var XPATH_FIRST_ORDERED_NODE_TYPE = 9;
    
    var robotXml = xmlDoc.evaluate('//robot', xmlDoc, null, XPATH_FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
    var c = 0;
    var container = document.getElementById('sliders');
    for (var nodes = robotXml.childNodes, i = 0; i < nodes.length; i++) {
        var node = nodes[i];
        if(node.tagName==='joint'){
            if(node.getAttribute('type')!=='fixed'){
  
                var minval, maxval, val;
                if(node.getAttribute('type')==='continuous'){
                    minval = -3.1415;
                    maxval = 3.1415;
                    val = 0;
                }else{
                    var limit = node.getElementsByTagName('limit')[0];
                    minval = parseFloat(limit.getAttribute('lower'));
                    maxval = parseFloat(limit.getAttribute('upper'));
                    if(minval <= 0 && maxval >= 0){
                        val = 0;
                    }else{
                        val = (maxval + minval) / 2;
                    } 
                }           
            
                var name = node.getAttribute('name');
                var x = document.createTextNode( name );
                container.appendChild(x);
                x = document.createElement('input');
                x.setAttribute('name', name + '_text');
                x.setAttribute('id', name + '_text');
                x.setAttribute('style', 'float: right');
                x.setAttribute('value', val);
                x.onblur = updateInput;
                container.appendChild(x);
                container.appendChild( document.createElement('br') );
                
                x = document.createElement('input');
                x.setAttribute('name', name);
                x.setAttribute('id', name + '_slider');
                x.setAttribute('type', 'range');
                x.setAttribute('min', minval);
                x.setAttribute('max', maxval);
                x.setAttribute('value', val);
                x.setAttribute('step', (maxval-minval)/100);
                x.setAttribute('style', 'width: 100%');
                x.onchange = updateInput;
                container.appendChild(x);
                container.appendChild( document.createElement('br') );
                sliders[ sliders.length ] = x;
            }
        }
    }  
  };
  
  param.get(load_model);
    
  var topic = new ROSLIB.Topic({
    ros : ros,
    name : topicName,
    messageType : 'sensor_msgs/JointState'
  });
  
  function publish()
  {
      var names = [];
      var values = [];
      for(var index = 0; index < sliders.length; index++) {
        var slider = sliders[index];
        names[ names.length ] = slider.name;
        values[ values.length ] = parseFloat(slider.value);
      }

      var js = new ROSLIB.Message({
        name: names, position: values
      });
      topic.publish(js);
  }


};




