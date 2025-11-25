import plotly.express as px
import plotly.graph_objects as go
import plotly.figure_factory as ff
from plotly.subplots import make_subplots
import pandas as pd
import os

"""

THIS PROGRAM DOES NOT WORK ON THE RPi BECAUSE OF WRONG PLOTLY VERSION... TO USE THIS PROGRAM TO GENERATE GRAPHS -> 
COPY THE PROGRAM TO PC ALONG WITH THE "RUN DATA" FOLDER AND GENERATE THE GRAPHS ON THE PC.

"""

output_path = "./Output/"


def _prepare_folders(morphology):
    """
    Function that creates folders containing the output graphs in the current folder
    :return: None
    """

    this_run_path = f'{output_path}/{morphology}'

    # Prepare the folders to save graphs in
    os.makedirs(this_run_path, exist_ok=True)

    # Prepare unique folder for the current graph
    if not os.path.exists(f"{this_run_path}/Run_1"):
        this_run_path = f"{this_run_path}/Run_1"

    else:
        existing_runs = [x[1] for x in os.walk(this_run_path)][0]
        existing_indexes = []
        for item in existing_runs:
            list_of_indexes = [i for i in item if i.isdigit()]
            index = int("".join(list_of_indexes))
            existing_indexes.append(index)

        last_run_index = max(existing_indexes)
        this_run_path = f"{this_run_path}/Run_{last_run_index+1}"

    os.mkdir(this_run_path)
    return this_run_path


cols = getattr(px.colors.sequential, 'Magma')
col_l = len(cols)


def _create_run_parameters_txt(this_run_path, hexapod, joints):
    """
    Function that creates a text file summarizing the parameters of the hexapod and its joints during the current run.
    :param this_run_path: Path to the current run folder
    :param hexapod: Loaded hexapod data
    :param joints: Loaded joints data
    :return: None
    """
    this_run_index = [int(i) for i in this_run_path if i.isdigit()][-1]
    with open(f"{this_run_path}/Run_{this_run_index}_parameters.txt", "w") as f:
        hexapod_parameters = hexapod.get_hexapod_run_parameters()
        personal_comment = hexapod_parameters.popitem()[1]

        text = f'Platform: {joints[0].platform}\n\nHEXAPOD PARAMETERS:\n\n'
        for item in hexapod_parameters:
            text += f'      {item}: {hexapod_parameters[item]}\n'

        text += '\n'
        text += 'JOINT UNIQUE PARAMETERS:\n\n'
        for joint in joints:
            print(f"Creating {joint.name} run parameter file...")
            joint_parameters = joint.get_joint_run_parameters()
            for item in joint_parameters:
                if joint_parameters[item] != hexapod_parameters[item]:
                    text += f'  {joint.name}:\n'
                    break
            for item in joint_parameters:
                if joint_parameters[item] != hexapod_parameters[item]:
                    text += f'      {item}: {joint_parameters[item]}\n'
        text += '\n\n\nPersonal comment:\n\n'
        text += f'      {personal_comment}'
        f.write(text)


def _create_raw_data_txt(this_run_path, joints):
    """
    Function that creates a text file where only the measured data is outputted.
    :param this_run_path: Path to the current run folder
    :param joints: Loaded joints data
    :return:
    """
    os.makedirs(f"{this_run_path}/JointRawData", exist_ok=True)
    for joint in joints:
        if joint.type == "Knee" or joint.type == "Shoulder":
            print(f"Creating {joint.name} raw data files...")
            data = list(zip(joint.diagnose_times, joint.position_history))
            text = ""
            with open(f"{this_run_path}/JointRawData/{joint.name}_positions.txt", "w") as a:
                for time, position in data:
                    text += f'{round(time, 3)} {position}\n'
                a.write(text)
                a.close()
            data = list(zip(joint.diagnose_times, joint.voltage_history))
            text = ""
            with open(f"{this_run_path}/JointRawData/{joint.name}_voltages.txt", "w") as b:
                for time, voltage in data:
                    text += f'{round(time, 3)} {voltage[1]}\n'
                b.write(text)
                b.close()
            data = list(zip(joint.diagnose_times, joint.state_history))
            text = ""
            with open(f"{this_run_path}/JointRawData/{joint.name}_states.txt", "w") as c:
                for time, state in data:
                    text += f'{round(time, 3)} {state}\n'
                c.write(text)
                c.close()
            data = list(zip(joint.diagnose_times, joint.voltage_history, joint.state_history))
            text = ""
            if len(data) > 0:
                reference_voltage = data[0][1][1]
                current_state = data[0][2]
            else:
                reference_voltage = 69
                current_state = 'None'
            with open(f"{this_run_path}/JointRawData/{joint.name}_states_analysis.txt", "w") as c:
                for time, voltage, state in data:
                    if state != current_state:
                        reference_voltage = voltage[1]
                        current_state = state

                    voltage_diff = reference_voltage - voltage[1]
                    if state == 'TD' and abs(voltage_diff) > joint.down_v_threshold:
                        switch_state = True
                    elif state == 'LO' and abs(voltage_diff) > joint.up_v_threshold:
                        switch_state = True
                    else:
                        switch_state = False
                    text += f't = {round(time, 3)} {state} rV = {reference_voltage} cV = {voltage[1]} dV = {voltage_diff}       switch = {switch_state}\n'
                c.write(text)
                c.close()


def _process_leg_states(joint):
    """
    Function that reorganizes the measured data in order to be able to generate the footfall pattern timeline graph.
    :param joint: Loaded joint data.
    :return:
    """
    if "Knee" in joint.type:
        diagnose_times = [round(i, 2) for i in joint.diagnose_times]
        data = list(zip(diagnose_times, joint.state_history))
        output = []
        state = data[0][1]
        state_start_time = data[0][0]
        for i in range(0, len(data)):
            if data[i][1] != state:
                state_end_time = data[i][0]
                output.append([state_start_time, state_end_time, state])
                state = data[i][1]
                state_start_time = data[i][0]

        return output


def complete_joint_overview(joints: list, hexapod):
    """
    Function that creates Plotly graph, plotting the joints position and voltage history for every joint in one figure.
    Outputs: ".html" file.
    :param hexapod: Loaded hexapod object
    :param joints: List of all joint objects contained in the hexapod object.
    :return: None
    """

    this_run_path = _prepare_folders(hexapod.morphology.name)
    this_run_index = [int(i) for i in f"{this_run_path}/" if i.isdigit()][-1]
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True)
    title = f"Run {this_run_index} Joint Overview"

    for joint in joints:
        # Remove initial diagnosis
        joint.diagnose_times.pop(0)
        joint.position_history.pop(0)
        joint.voltage_history.pop(0)
        joint.state_history.pop(0)

    all_positions = []
    all_raw_voltages = []
    all_filtered_voltages = []
    all_states = []

    _create_run_parameters_txt(this_run_path, hexapod, joints)
    _create_raw_data_txt(this_run_path, joints)
    _footfall_pattern_graph(this_run_path, joints, hexapod.morphology)

    # Iterate over every joint.
    for n, joint in enumerate(joints):
        if joint.name == "LF Knee" or joint.name == "LF Shoulder":
            print(f"Plotting {joint.name} plot...")
            t = [i for i in joint.diagnose_times]
            all_positions.append(joint.position_history)
            all_states.append(joint.state_history)
            # Plot the current joint in the figure
            fig.add_trace(go.Scatter(
                x=t,
                y=joint.position_history,
                line=dict(color=cols[n % col_l]),
                mode="lines+markers",
                name=f"{joint.name}",
                text=joint.state_history,
                textposition="top center",
                visible="legendonly",
                legendgroup=n,
                textfont=dict(color=cols[n % col_l])
            ), row=1, col=1)
            fig.update_xaxes(title_text="Time [s]", row=1, col=1)
            fig.update_yaxes(title_text="Positions [-]", row=1, col=1)

            raw_voltage = [i[0] for i in joint.voltage_history]
            filtered_voltage = [i[1] for i in joint.voltage_history]
            all_raw_voltages.append(raw_voltage)
            all_filtered_voltages.append(filtered_voltage)

            # Plot the current joint in the figure
            fig.add_trace(go.Scatter(
                x=t,
                y=raw_voltage,
                line=dict(color=cols[n % col_l]),
                mode="lines+markers",
                name=f"{joint.name} V",
                text=joint.state_history,
                textposition="top center",
                visible="legendonly",
                showlegend=False,
                legendgroup=n,
                textfont=dict(color=cols[n % col_l])
            ), row=2, col=1)

            fig.add_trace(go.Scatter(
                x=t,
                y=filtered_voltage,
                line=dict(color=cols[(n + 5) % col_l]),
                mode="lines+markers",
                name=f"{joint.name} V",
                visible="legendonly",
                showlegend=False,
                legendgroup=n,
            ), row=2, col=1)

            fig.update_xaxes(title_text="Steps", row=1, col=1)
            fig.update_yaxes(title_text="Voltages [mV]", row=2, col=1)

            leg_states = _process_leg_states(joint)
            color = {"TD": "lightcyan",
                     "ST": "lightcoral",
                     "LO": "papayawhip",
                     "SW": "indigo"}
            # Generate color background based on state
            if leg_states is not None:
                for item in leg_states:
                    fig.add_vrect(
                        x0=item[0], x1=item[1],
                        fillcolor=color[item[2]], opacity=0.5,
                        layer="below", line_width=0,
                        label=dict(text=item[2], textposition="top center"),
                        visible="legendonly", legendgroup=n, showlegend=True
                    ),

        fig.update_layout(
            title=f'Morphology: {hexapod.morphology.name} - {title}',
            legend_title="Legs:",
            font=dict(family="Courier New, monospace", size=18, color=cols[0]),
            template='ggplot2')

        # Add dropdown
        fig.update_layout(updatemenus=[
            dict(
                type="buttons",
                direction="left",
                buttons=list([dict(
                    args=[{"y": [all_positions[0],
                                 all_raw_voltages[0],
                                 all_filtered_voltages[0],
                                 ]}],
                    label="Both",
                    method="update"
                ),
                    dict(
                        args=[{"y": [all_positions[0],
                                     all_filtered_voltages[0],
                                     all_filtered_voltages[0],
                                     ]}],
                        label="Filtered Voltage",
                        method="update"
                    ),
                    dict(
                        args=[{"y": [all_positions[0],
                                     all_raw_voltages[0],
                                     all_raw_voltages[0],
                                     ]}],
                        label="Raw Voltage",
                        method="update"
                    ),

                ]),
                pad={"r": 10, "t": 10},
                showactive=True,
                x=0,
                xanchor="left",
                y=-0.1,
                yanchor="top"),
        ])
    title = f"{this_run_path}/{title}.html"
    # Save the figure as ".html" file and enable zooming by mouse wheel
    fig.write_html(title, config={'scrollZoom': True})
    return title

    # fig.show(config={'scrollZoom': True})  # Uncomment to open the created figure'''


def _footfall_pattern_graph(this_run_path, joints, morphology):
    """
    Function that creates the footfall pattern graph
    :param this_run_path: Path to the current run folder.
    :param joints: Loaded joint data.
    :param morphology: Morphology used for the current run.
    :return:
    """
    knees = []
    # Extract knees from joints
    for joint in joints:
        if joint.type == "Knee":
            knees.append(joint)
    knees = knees[0], knees[2], knees[1], knees[3], knees[5], knees[4]  # Reorder knees
    footfalls = []

    # Create dict containing footfalls
    for joint in knees:
        leg_states = _process_leg_states(joint)
        if leg_states is not None:
            only_td = [i for i in leg_states if i[2] == "ST"]
            for td in only_td:
                footfalls.append(dict(Task=joint.name[:2], Start=td[0], Finish=td[1], Resource=joint.name[1:2]))

    df = pd.DataFrame(footfalls)
    fig = ff.create_gantt(df, bar_width=0.4, show_colorbar=True, index_col='Resource',  colors=px.colors.qualitative.Set1, group_tasks=True)
    fig.update_layout(xaxis_type='linear', showlegend=False, template='ggplot2', title=f"{morphology.name} - Footfall Pattern")
    fig.write_html(f"{this_run_path}/{morphology.name} footfalls.html", config={'scrollZoom': True})
    # fig.show()


if __name__ == '__main__':
    from hexapod_class import Joint, Hexapod
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()

    # Select either specific run or the RunData directory to generate graphs
    folder_path = filedialog.askdirectory(initialdir='RunData/')
    loaded_hexapod = None
    where = None
    print("Generating the files...")
    # Generate graphs for each run in the RunData directory if selected
    if os.path.basename(os.path.normpath(folder_path)) == 'RunData':
        for run in os.listdir(folder_path):
            loaded_joints = []
            loaded_hexapod = Hexapod.load_hexapod(f'{folder_path}/{run}/Hexapod')
            for joint_file in os.listdir(f'{folder_path}/{run}/JointData'):
                filename = os.fsdecode(joint_file)
                file_path = f'{folder_path}/{run}/JointData/{filename}'
                loaded_joints.append(Joint.load_joint(file_path))

            where = complete_joint_overview(loaded_joints, loaded_hexapod)

    # Generate graphs for specific run
    else:
        loaded_joints = []
        loaded_hexapod = Hexapod.load_hexapod(f'{folder_path}/Hexapod')
        for joint_file in os.listdir(f'{folder_path}/JointData'):
            filename = os.fsdecode(joint_file)
            file_path = f'{folder_path}/JointData/{filename}'
            loaded_joints.append(Joint.load_joint(file_path))

        where = complete_joint_overview(loaded_joints, loaded_hexapod)

    print(f"Files have been successfully generated! at {where}")
