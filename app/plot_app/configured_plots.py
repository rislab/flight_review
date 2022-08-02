""" This contains the list of all drawn plots on the log plotting page """

import matplotlib.pyplot as plt
from html import escape

from bokeh.layouts import column
from bokeh.models import Range1d
from bokeh.models.widgets import Button
from bokeh.io import curdoc

from config import *
from helper import *
from leaflet import ulog_to_polyline
from plotting import *
from plotted_tables import (
    get_logged_messages, get_changed_parameters,
    get_info_table_html, get_heading_html, get_error_labels_html,
    get_hardfault_html, get_corrupt_log_html
    )

#pylint: disable=cell-var-from-loop, undefined-loop-variable,
#pylint: disable=consider-using-enumerate,too-many-statements



def generate_plots(ulog, ulog2, px4_ulog, px4_ulog2, db_data, vehicle_data, link_to_3d_page,
                   link_to_pid_analysis_page):
    """ create a list of bokeh plots (and widgets) to show """

    plots = []
    data = ulog.data_list
    data2 = ulog2.data_list

    # COMPATIBILITY support for old logs
    if any(elem.name in ('vehicle_air_data', 'vehicle_magnetometer') for elem in data):
        baro_alt_meter_topic = 'vehicle_air_data'
        magnetometer_ga_topic = 'vehicle_magnetometer'
    else: # old
        baro_alt_meter_topic = 'sensor_combined'
        magnetometer_ga_topic = 'sensor_combined'
    for topic in data:
        if topic.name == 'system_power':
            # COMPATIBILITY: rename fields to new format
            if 'voltage5V_v' in topic.data:     # old (prior to PX4/Firmware:213aa93)
                topic.data['voltage5v_v'] = topic.data.pop('voltage5V_v')
            if 'voltage3V3_v' in topic.data:    # old (prior to PX4/Firmware:213aa93)
                topic.data['sensors3v3[0]'] = topic.data.pop('voltage3V3_v')
            if 'voltage3v3_v' in topic.data:
                topic.data['sensors3v3[0]'] = topic.data.pop('voltage3v3_v')
        if topic.name == 'tecs_status':
            if 'airspeed_sp' in topic.data: # old (prior to PX4-Autopilot/pull/16585)
                topic.data['true_airspeed_sp'] = topic.data.pop('airspeed_sp')

    if any(elem.name == 'vehicle_angular_velocity' for elem in data):
        rate_estimated_topic_name = 'vehicle_angular_velocity'
        rate_groundtruth_topic_name = 'vehicle_angular_velocity_groundtruth'
        rate_field_names = ['xyz[0]', 'xyz[1]', 'xyz[2]']
    else: # old
        rate_estimated_topic_name = 'vehicle_attitude'
        rate_groundtruth_topic_name = 'vehicle_attitude_groundtruth'
        rate_field_names = ['rollspeed', 'pitchspeed', 'yawspeed']
    if any(elem.name == 'manual_control_switches' for elem in data):
        manual_control_switches_topic = 'manual_control_switches'
    else: # old
        manual_control_switches_topic = 'manual_control_setpoint'
    dynamic_control_alloc = any(elem.name in ('actuator_motors', 'actuator_servos')
                                for elem in data)
    actuator_controls_0 = ActuatorControls(ulog, dynamic_control_alloc, 0)
    actuator_controls_1 = ActuatorControls(ulog, dynamic_control_alloc, 1)


    # Position plot
    data_plot = DataPlot2D(data, plot_config, 'vehicle_local_position',
                           x_axis_label='[m]', y_axis_label='[m]', plot_height='large')
    data_plot.add_graph('y', 'x', colors2[0], 'Estimated',
                        check_if_all_zero=True)
    if not data_plot.had_error: # vehicle_local_position is required
        data_plot.change_dataset('vehicle_local_position_setpoint')
        data_plot.add_graph('y', 'x', colors2[1], 'Setpoint')
        # groundtruth (SITL only)
        data_plot.change_dataset('vehicle_local_position_groundtruth')
        data_plot.add_graph('y', 'x', color_gray, 'Groundtruth')
        # GPS + position setpoints
        plot_map(ulog, plot_config, map_type='plain', setpoints=True,
                 bokeh_plot=data_plot.bokeh_plot)
        if data_plot.finalize() is not None:
            plots.append(data_plot.bokeh_plot)


    # initialize parameter changes
    changed_params = None
    if not 'replay' in ulog.msg_info_dict: # replay can have many param changes
        if len(ulog.changed_parameters) > 0:
            changed_params = ulog.changed_parameters
            plots.append(None) # save space for the param change button

    ### Add all data plots ###
    x_range_offset = (ulog.last_timestamp - ulog.start_timestamp) * 0.05
    x_range = Range1d(ulog.start_timestamp - x_range_offset, ulog.last_timestamp + x_range_offset)

    # magnetic field norm
    data_plot = DataPlot(data, plot_config, magnetometer_ga_topic,
                         title='Norm Magnetic Field', plot_height='small',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph(
        [lambda data: ('len_mag', np.sqrt(data['magnetometer_ga[0]']**2 +
                                          data['magnetometer_ga[1]']**2 +
                                          data['magnetometer_ga[2]']**2))],
        colors3[0:1], ['#1'])
    data_plot = DataPlot(data2, plot_config, magnetometer_ga_topic,
                         title='Norm Magnetic Field', plot_height='small',
                         changed_params=changed_params, x_range=x_range, p=data_plot._p)
    data_plot.add_graph(
        [lambda data: ('len_mag', np.sqrt(data['magnetometer_ga[0]']**2 +
                                          data['magnetometer_ga[1]']**2 +
                                          data['magnetometer_ga[2]']**2))],
        colors3[2:3], ['#2'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    # gps uncertainty
    # the accuracy values can be really large if there is no fix, so we limit the
    # y axis range to some sane values
    data_plot = DataPlot(data, plot_config, 'vehicle_gps_position',
                         title='GPS Uncertainty', y_range=Range1d(0, 40),
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['eph', 'epv', 'satellites_used', 'fix_type'], colors8[::2],
                        ['#1 Horizontal position accuracy [m]', '#1 Vertical position accuracy [m]',
                         '#1 Num Satellites used', '#1 GPS Fix'])
    data_plot = DataPlot(data2, plot_config, 'vehicle_gps_position',
                         title='GPS Uncertainty', y_range=Range1d(0, 40),
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range, p=data_plot._p)
    data_plot.add_graph(['eph', 'epv', 'satellites_used', 'fix_type'], colors8_extra[::2],
                        ['#2 Horizontal position accuracy [m]', '#2 Vertical position accuracy [m]',
                         '#2 Num Satellites used', '#2 GPS Fix'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    # exchange all DataPlot's with the bokeh_plot and handle parameter changes
    param_changes_button = Button(label="Hide Parameter Changes", width=170)
    param_change_labels = []
    # FIXME: this should be a CustomJS callback, not on the server. However this
    # did not work for me.
    def param_changes_button_clicked():
        """ callback to show/hide parameter changes """
        for label in param_change_labels:
            if label.visible:
                param_changes_button.label = 'Show Parameter Changes'
                label.visible = False
                label.text_alpha = 0 # label.visible does not work, so we use this instead
            else:
                param_changes_button.label = 'Hide Parameter Changes'
                label.visible = True
                label.text_alpha = 1
    param_changes_button.on_click(param_changes_button_clicked)


    jinja_plot_data = []
    for i in range(len(plots)):
        if plots[i] is None:
            plots[i] = column(param_changes_button, width=int(plot_width * 0.99))
        if isinstance(plots[i], DataPlot):
            if plots[i].param_change_label is not None:
                param_change_labels.append(plots[i].param_change_label)

            plot_title = plots[i].title
            plots[i] = plots[i].bokeh_plot

            fragment = 'Nav-'+plot_title.replace(' ', '-') \
                .replace('&', '_').replace('(', '').replace(')', '')
            jinja_plot_data.append({
                'model_id': plots[i].ref['id'],
                'fragment': fragment,
                'title': plot_title
                })


    # changed parameters
    plots.append(get_changed_parameters(ulog, plot_width))

    # log messages
    plots.append(get_logged_messages(ulog, plot_width))


    # console messages, perf & top output
    top_data = ''
    perf_data = ''
    console_messages = ''
    if 'boot_console_output' in ulog.msg_info_multiple_dict:
        console_output = ulog.msg_info_multiple_dict['boot_console_output'][0]
        console_output = escape(''.join(console_output))
        console_messages = '<p><pre>'+console_output+'</pre></p>'

    for state in ['pre', 'post']:
        if 'perf_top_'+state+'flight' in ulog.msg_info_multiple_dict:
            current_top_data = ulog.msg_info_multiple_dict['perf_top_'+state+'flight'][0]
            flight_data = escape('\n'.join(current_top_data))
            top_data += '<p>'+state.capitalize()+' Flight:<br/><pre>'+flight_data+'</pre></p>'
        if 'perf_counter_'+state+'flight' in ulog.msg_info_multiple_dict:
            current_perf_data = ulog.msg_info_multiple_dict['perf_counter_'+state+'flight'][0]
            flight_data = escape('\n'.join(current_perf_data))
            perf_data += '<p>'+state.capitalize()+' Flight:<br/><pre>'+flight_data+'</pre></p>'
    if 'perf_top_watchdog' in ulog.msg_info_multiple_dict:
        current_top_data = ulog.msg_info_multiple_dict['perf_top_watchdog'][0]
        flight_data = escape('\n'.join(current_top_data))
        top_data += '<p>Watchdog:<br/><pre>'+flight_data+'</pre></p>'

    additional_data_html = ''
    if len(console_messages) > 0:
        additional_data_html += '<h5>Console Output</h5>'+console_messages
    if len(top_data) > 0:
        additional_data_html += '<h5>Processes</h5>'+top_data
    if len(perf_data) > 0:
        additional_data_html += '<h5>Performance Counters</h5>'+perf_data
    if len(additional_data_html) > 0:
        # hide by default & use a button to expand
        additional_data_html = '''
<button id="show-additional-data-btn" class="btn btn-secondary" data-toggle="collapse" style="min-width:0;"
 data-target="#show-additional-data">Show additional Data</button>
<div id="show-additional-data" class="collapse">
{:}
</div>
'''.format(additional_data_html)
        curdoc().template_variables['additional_info'] = additional_data_html


    curdoc().template_variables['plots'] = jinja_plot_data

    return plots
