# Directive that shows a toggle link between sections.
# Those sections cannot contain titles
# The first one is initially showed. Here is the syntax:
# .. toggle_table::
#     :arg1: text in button 1
#     :arg2: text in button 2
#
# .. toggle:: text in button 1
#
#    some RST text
#
#.. toggle:: text in button 2
#
#    some more RST text
import docutils
from docutils import nodes
from docutils.parsers.rst import directives, roles, states

def setup(app):
    app.add_directive('toggle', ToggleDirective)
    app.add_directive('toggle_table', ToggleTableDirective)

class ToggleDirective(docutils.parsers.rst.Directive):
    """
    Base class that will be used by the two toggle directives
    """
    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = True
    option_spec = {}
    has_content = True
    node_class = nodes.container

    def run(self):
        self.assert_has_content()
        text = '\n'.join(self.content)
        # Create the node, to be populated by `nested_parse`.
        node = self.node_class(rawsource=text)
        label = self.arguments[0]
        label_strip = label.replace(' ', '')
        node += nodes.raw(self.arguments[0], '<div class="toggleable_div label_%s">' % label_strip, format="html")

        # Parse the directive contents.
        self.state.nested_parse(self.content, self.content_offset, node)
        node += nodes.raw(self.arguments[0], '</div>', format="html")
        return [node]

class ToggleTableDirective(docutils.parsers.rst.Directive):
    """
    Class used to create a set of buttons to toggle different sections
    """
    required_arguments = 0
    optional_arguments = 10
    final_argument_whitespace = True
    option_spec = {}
    for i in xrange(0, 100):
        option_spec['arg' + str(i)] = str
    has_content = True
    node_class = nodes.container

    def run(self):
        js_toggle = """
function toggle(label) {
  $('.toggleable_button').css({border: '2px outset', 'border-radius': '4px'});
  $('.toggleable_button.label_' + label).css({border: '2px inset', 'border-radius': '4px'});
  $('.toggleable_div').css('display', 'none');
  $('.toggleable_div.label_' + label).css('display', 'block');
};
"""

        js_ready = """
<script>
%s
$(document).ready(function() {
  var classList =$('.toggleable_button').attr('class').split(/\s+/);
  $.each( classList, function(index, item){
    if (item.substring(0, 5) === 'label') {
      toggle(item.substring(6));
    };
  });
});
</script>
""" % js_toggle

        # Create the node, to be populated by `nested_parse`.
        node = self.node_class()
        for key in self.options.keys():
            if key not in self.option_spec:
                raise RuntimeError(key + ' not in the contructor of ToggleTableDirective, use arg0 to arg99')
            label = self.options[key]
            label_strip = label.replace(' ', '')
            str1 = '<button class="toggleable_button label_%s" onclick="' % label_strip
            str2 = js_toggle + "toggle('%s')" % label_strip
            str3 = '">%s</button>' % label
            node += nodes.raw(key, str1 + str2 + str3 + js_ready, format="html")

        return [node]
