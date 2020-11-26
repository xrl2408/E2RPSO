# E2RPSO
Exploration Enhanced RPSO for Collaborative Multi-target Searching of Robotic Swarms
# Usage
### E2RPSO
'''
import E2RPSO
E2RPSO()

'''
### MLPSO
'''
import MLPSO
MLPSO()

'''
### RDPSO
'''
import RDPSO
RDPSO()

'''
### RPSO
'''
import RPSO
RPSO()

'''
# Configuration
### E2RPSO
|Parameters|Describe|Value|
|PBEST|Initialize Pbest|2000000000|
|GBEST|Initialize Gbest|2000000000|
|save|Safety distance of obstacle avoidance|1|
|show_animation|Draw trajectory|True|
|show_type|1 Static diagram, 2 dynamic diagram|1 or 2|
|can_print|print running information|1|
|save_path|Store path information|False or True|
|Path|Storage path|-|
|V_LIMIT|Max velocity in single iteration|2|
|T|Max iteration times|10000|
|C1|parameter for Pbest|0.4|
|C2|parameter for Gbest|0.9|
|C3_index|avoid local optimum|5|
|C4_index|avoid obstancle|10|
|K|Map area exploration rate increment|1|
|SIDE|map side|1000|
|Size|robots num|20|
|Goal_num|goal num|10|

### MLPSO
|Parameters|Describe|Value|
|PBEST|Initialize Pbest|2000000000|
|GBEST|Initialize Gbest|2000000000|
|save|Safety distance of obstacle avoidance|1|
|show_animation|Draw trajectory|True|
|show_type|1 Static diagram, 2 dynamic diagram|1 or 2|
|can_print|print running information|1|
|save_path|Store path information|False or True|
|Path|Storage path|-|
|V_LIMIT|Max velocity in single iteration|2|
|T|Max iteration times|10000|
|C1|parameter for Pbest|0.4|
|C2|parameter for Gbest|0.9|
|C3_index|avoid local optimum|5|
|C4_index|avoid obstancle|10|
|K|Map area exploration rate increment|1|
|SIDE|map side|1000|
|Size|robots num|20|
|Goal_num|goal num|10|

### RDPSO
|Parameters|Describe|Value|
|PBEST|Initialize Pbest|2000000000|
|GBEST|Initialize Gbest|2000000000|
|save|Safety distance of obstacle avoidance|1|
|show_animation|Draw trajectory|True|
|show_type|1 Static diagram, 2 dynamic diagram|1 or 2|
|can_print|print running information|1|
|save_path|Store path information|False or True|
|Path|Storage path|-|
|V_LIMIT|Max velocity in single iteration|2|
|T|Max iteration times|10000|
|C1|parameter for Pbest|0.4|
|C2|parameter for Gbest|0.9|
|C3|avoid obstancle|100000|
|SIDE|map side|1000|
|Size|robots num|20|
|Goal_num|goal num|10|

### RPSO
|Parameters|Describe|Value|
|PBEST|Initialize Pbest|2000000000|
|GBEST|Initialize Gbest|2000000000|
|save|Safety distance of obstacle avoidance|1|
|show_animation|Draw trajectory|True|
|show_type|1 Static diagram, 2 dynamic diagram|1 or 2|
|can_print|print running information|1|
|save_path|Store path information|False or True|
|Path|Storage path|-|
|V_LIMIT|Max velocity in single iteration|2|
|T|Max iteration times|10000|
|C1|parameter for Pbest|0.4|
|C2|parameter for Gbest|0.9|
|C3|avoid obstancle|100000|
|SIDE|map side|1000|
|Size|robots num|20|
|Goal_num|goal num|10|


