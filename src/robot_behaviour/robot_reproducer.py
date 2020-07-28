'''
IF CORRECT ACTION HAS BEEN PERFORMED
'''
def congratulate(speech, face=None, gesture=None):
  '''
  It reproduces the audio, face and gesture provided as input
  Args:
    speech:
    face:
    gesture:
  Return:
     Bool: if the action has been executed successfully
  '''
  print("Congrats funct")
  b_executed = False
  return b_executed

'''
IF WRONG ACTION HAS BEEN PERFORMED 
'''
def compassion(speech, face=None, gesture=None):
  '''
    It reproduces the audio, face and gesture provided as input
    Args:
      speech:
      face:
      gesture:
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Compassion funct")
  b_executed = False
  return b_executed


def move_token_back(who, token_from, token_to, speech, face=None, gesture=None):
  '''
    It reproduces the audio, face and gesture provided as input.
    The robot will pick the wrong token and move it back to its initial location
    Args:
      speech:
      face:
      gesture:
      who: if the robot has to move back the token or the user
      token_from: the current loc of the token
      token_to: the loc where the token has to be moved
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Move token back")
  b_executed = False
  return b_executed
'''
LEVELS OF ASSISTANCE FURNISHED BY THE ROBOT
'''

def no_assistance(speech, face=None, gesture=None):
  '''
  It provides no assistance to the user, basically only inform the user of moving a token
  Args:
    speech:
    face:
    gesture:
  Return:
  '''
  print("Lev_0")
  b_executed = False
  return b_executed

def encouragement(speech, face=None, gesture=None):
  '''
  It eoncurages the user to move a token
    Args:
      speech:
      face:
      gesture:
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_1")
  b_executed = False
  return b_executed

def suggest_row(row, speech, face=None, gesture=None):
  '''
  It tells the user the row of the board where the correct token is
    Args:
      speech:
      face:
      gesture:
      row: the row of the board where the robot has to point
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_2")
  b_executed = False
  return b_executed

def suggest_cells(speech, face=None, gesture=None, *tokens):
  '''
  It tells the user the cells near to the correct token (included the latter)
    Args:
      speech:
      face:
      gesture:
      tokens: the tokens on the board where the robot has to point
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_3")
  b_executed = False
  return b_executed

def suggest_solution(speech, face=None, gesture=None, *token):
  '''
  It tells the user the cell where the correct token is
    Args:
      speech:
      face:
      gesture:
      token: The correct token, id and location
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_4")
  b_executed = False
  return b_executed

def offer_solution(speech, face=None, gesture=None, *token):
  '''
  It offers the user the correct token
    Args:
      speech:
      face:
      gesture:
      token: The correct token, id and location
    Return:
       Bool: if the action has been executed successfully
    '''
  print("Lev_5")
  b_executed = False
  return b_executed


robot_actions = {
  "congrats": congratulate,
  "compassion": compassion,
  "move_back": move_token_back,
  "lev_0": no_assistance,
  "lev_1": encouragement,
  "lev_2": suggest_row,
  "lev_3": suggest_cells,
  "lev_4": suggest_solution,
  "lev_5": offer_solution
}

speech = ""; face=""; gesture=""; who=""; token_from=""; token_to=""; row=""; cells=["a", "b"]; token="";
robot_actions["congrats"].__call__(speech, face, gesture)
robot_actions["compassion"].__call__(speech, face, gesture)
#take in input 5 params
robot_actions["move_back"].__call__(who, token_from, token_to, speech, face, gesture)
robot_actions["lev_0"].__call__(speech, face, gesture)
robot_actions["lev_1"].__call__(speech, face, gesture)
robot_actions["lev_2"].__call__(row, speech, face, gesture)
robot_actions["lev_3"].__call__(speech,  *cells)
robot_actions["lev_4"].__call__(speech, face, gesture, token)
robot_actions["lev_5"].__call__(speech, face, gesture, token)

