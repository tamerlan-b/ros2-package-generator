from graphviz import Digraph
from typing import Dict

def draw_node(node_config: Dict) -> Digraph:
        # Create graph
        dot = Digraph(comment='ROS2 Node', format='svg')
        dot.attr(rankdir='LR')  # Left to Right
        dot.attr('node', fontname='Arial', fontsize='11')
        dot.attr('edge', arrowhead='normal')
        
        node_color = 'lightblue2'
        node_border_color = 'navy'
        topic_color='lightcoral'
        topic_border_color = 'darkred'
        sync_color = 'plum'
        sync_border_color = 'purple'
        service_color='lemonchiffon'
        service_border_color = 'darkyellow'
        action_color='lightgreen'
        action_border_color = 'darkgreen'
        
        unknown = '???'
        
        # === Node ===
        node_name = node_config.get('node_name', 'unnamed_node')
        dot.node('NODE', 
                f'<<B>{node_name}</B><BR/><FONT POINT-SIZE="9">Node</FONT>>', 
                shape='box', style='filled', fillcolor=node_color, 
                color=node_border_color, fontsize='16')
        
        # === INPUTS (to the left) ===
        
        # Subscribers
        for i, sub in enumerate(node_config.get('subscribers', [])):
            node_id = f'SUB_{i}'
            topic_name = sub.get('topic', unknown)
            dot.node(node_id, 
                    f'<<B>{topic_name}</B><BR/><FONT POINT-SIZE="8">{sub.get("msg_type", unknown)}</FONT>>',
                    shape='parallelogram', style='filled', 
                    fillcolor=topic_color, color=topic_border_color)
            dot.edge(node_id, 'NODE', color=topic_border_color)
        
        # Service clients
        for i, client in enumerate(node_config.get('clients', [])):
            node_id = f'CLIENT_{i}'
            srv_name = client.get('srv_name', unknown)
            dot.node(node_id, 
                    f'<<B>{srv_name}</B><BR/><FONT POINT-SIZE="8">{client.get("type", unknown)}</FONT>>',
                    shape='parallelogram', style='filled', 
                    fillcolor=service_color, color=service_border_color)
            dot.edge(node_id, 'NODE', color=service_border_color)

        # Action clients
        for i, action_client in enumerate(node_config.get('action_clients', [])):
            node_id = f'ACT_CLIENT_{i}'
            srv_name = action_client.get('srv_name', unknown)
            dot.node(node_id, 
                    f'<<B>{srv_name}</B><BR/><FONT POINT-SIZE="8">{action_client.get("type", unknown)}</FONT>>',
                    shape='parallelogram', style='filled', 
                    fillcolor=action_color, color=action_border_color)
            dot.edge(node_id, 'NODE', color=action_border_color)
        
        # Synchronized subscribers
        for sync_idx, sync_group in enumerate(node_config.get('sync_subscribers', [])):
            group_id = f'SYNC_GROUP_{sync_idx}'
            with dot.subgraph(name=f'cluster_{group_id}') as cluster:
                sync_policy = sync_group.get('sync_policy', 'ApproximateTime')
                cluster.attr(style='dashed', color=sync_color, rank='same')
                sync_node_id = f'SYNC_{sync_idx}'
                cluster.node(sync_node_id,
                            f'<<B>Synchronizer</B><BR/><FONT POINT-SIZE="7">{sync_policy}</FONT>>',
                            shape='diamond', style='filled', 
                            fillcolor=sync_color, color=sync_border_color, fontsize='9')
                for sub_idx, sub in enumerate(sync_group.get('subs', [])):
                    topic_node_id = f'SYNC_TOPIC_{sync_idx}_{sub_idx}'
                    topic_name = sub.get('topic', unknown)
                    cluster.node(topic_node_id,
                            f'<<B>{topic_name}</B><BR/><FONT POINT-SIZE="8">{sub.get("msg_type", unknown)}</FONT>>',
                            shape='parallelogram', style='filled', 
                            fillcolor=topic_color, color=topic_border_color)
                    cluster.edge(topic_node_id, sync_node_id, color=topic_border_color, arrowhead='normal')
            dot.edge(f'SYNC_{sync_idx}', 'NODE', color='purple', fontsize='8')

        # === OUTPUTS (to the right) ===
        
        # Publishers
        for i, pub in enumerate(node_config.get('publishers', [])):
            node_id = f'PUB_{i}'
            topic_name = pub.get('topic', unknown)
            dot.node(node_id,
                    f'<<B>{topic_name}</B><BR/><FONT POINT-SIZE="8">{pub.get("msg_type", unknown)}</FONT>>',
                    shape='parallelogram', style='filled', 
                    fillcolor=topic_color, color=topic_border_color)
            dot.edge('NODE', node_id, color=topic_border_color)
        
        # Service servers
        for i, srv in enumerate(node_config.get('services', [])):
            node_id = f'SRV_{i}'
            srv_name = srv.get('name', unknown)
            dot.node(node_id,
                    f'<<B>{srv_name}</B><BR/><FONT POINT-SIZE="8">{srv.get("type", unknown)}</FONT>>',
                    shape='parallelogram', style='filled', 
                    fillcolor=service_color, color=service_border_color)
            dot.edge('NODE', node_id, color=service_border_color)
        
        # Action servers
        for i, action_srv in enumerate(node_config.get('action_servers', [])):
            node_id = f'ACT_SRV_{i}'
            srv_name = action_srv.get('name', unknown)
            dot.node(node_id,
                    f'<<B>{srv_name}</B><BR/><FONT POINT-SIZE="8">{action_srv.get("type", unknown)}</FONT>>',
                    shape='parallelogram', style='filled', 
                    fillcolor=action_color, color=action_border_color)
            dot.edge('NODE', node_id, color=action_border_color)
               
        # === LEGEND ===
        with dot.subgraph(name='cluster_legend') as legend:
            legend.attr(label='Legend', style='dotted', 
                    color='gray70', fontsize='9', rank='sink')
            
            legend_html = f'''<
            <TABLE BORDER="0" CELLBORDER="0" CELLSPACING="2">
                <TR><TD BGCOLOR="{topic_color}" WIDTH="20" HEIGHT="10"></TD><TD>Subscriber/Publisher</TD></TR>
                <TR><TD BGCOLOR="{service_color}" WIDTH="20" HEIGHT="10"></TD><TD>Service Client/Server</TD></TR>
                <TR><TD BGCOLOR="{action_color}" WIDTH="20" HEIGHT="10"></TD><TD>Action Client/Server</TD></TR>
                <TR><TD BGCOLOR="{node_color}" WIDTH="20" HEIGHT="10"></TD><TD>Node</TD></TR>
                <TR><TD BGCOLOR="{sync_color}" WIDTH="20" HEIGHT="10"></TD><TD>Syncronizer</TD></TR>
            </TABLE>
            >'''
            
            legend.node('LEGEND', legend_html, shape='plain')
        return dot