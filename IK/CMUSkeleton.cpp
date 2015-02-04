/*

Revision 1 - Steve Lin (CMU), Jan 14, 2002
Revision 2 - Alla Safonova and Kiran Bhat (CMU), Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao (USC), Feb, 2012

*/
#include <fstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "CMUSkeleton.h"


#ifdef WIN32
  #pragma warning(disable : 4996)
#endif

int CMUSkeleton::numBonesInSkel(Bone bone)
{
  Bone * tmp = bone.sibling;
  int numBones = 0;
  while (tmp != NULL) 
  {
    if (tmp->child != NULL)
      numBones += numBonesInSkel(*(tmp->child));
    numBones++; 
    tmp = tmp->sibling; 
  }
  if (bone.child != NULL)
    return numBones + 1 + numBonesInSkel(*bone.child);
  else
    return numBones + 1;
}

void CMUSkeleton::removeCR(char * str)
{
  if (str[strlen(str) - 1] == '\r')
    str[strlen(str) - 1] = 0;    
}

int CMUSkeleton::movBonesInSkel(Bone bone)
{
  Bone * tmp = bone.sibling;
  int numBones = 0;

  if (bone.dof > 0) numBones++;

  while (tmp != NULL) 
  {
    if (tmp->child != NULL)
      numBones += movBonesInSkel(*(tmp->child));
    if (tmp->dof > 0) 
    numBones++; 
    tmp = tmp->sibling; 
  }

  if (bone.child != NULL)
    return (numBones + movBonesInSkel(*bone.child));
  else
    return numBones;
}

// helper function to convert ASF part name into bone index
int CMUSkeleton::name2idx(char *name)
{
  int i=0;
  while(strcmp(m_pBoneList[i].name, name) != 0 && i++ < NUM_BONES_IN_ASF_FILE);
  return m_pBoneList[i].idx;
}

char * CMUSkeleton::idx2name(int idx)
{
  int i=0;
  while(m_pBoneList[i].idx != idx && i++ < NUM_BONES_IN_ASF_FILE);
  return m_pBoneList[i].name;
}

int CMUSkeleton::readASFfile(const char* asf_filename, double scale)
{
  //open file
  std::ifstream is(asf_filename, std::ios::in);
  if (is.fail()) 
    return -1;

  //
  // ignore header information
  //
  char str[2048], keyword[256];
  while (1)
  {
    is.getline(str, 2048);	
    removeCR(str);
    sscanf(str, "%s", keyword);
    if (strcmp(keyword, ":bonedata") == 0)	
      break;
  }

  //
  // read bone information: global orientation and translation, DOF.
  //
  is.getline(str, 2048);
  removeCR(str);
  char	part[256], *token;
  double length;

  bool done = false;
  for(int i = 1; (!done) && (i < MAX_BONES_IN_ASF_FILE); i++)
  {
    m_pBoneList[i].dof=0;
    m_pBoneList[i].dofrx = m_pBoneList[i].dofry = m_pBoneList[i].dofrz = 0;
    m_pBoneList[i].doftx = m_pBoneList[i].dofty = m_pBoneList[i].doftz = 0;
    m_pBoneList[i].doftl = 0;
    m_pBoneList[i].sibling = NULL; 
    m_pBoneList[i].child = NULL;
    NUM_BONES_IN_ASF_FILE++;
    MOV_BONES_IN_ASF_FILE++;
    while(1)
    {
      is.getline(str, 2048);	
      removeCR(str);
      sscanf(str, "%s", keyword);

      if(strcmp(keyword, "end") == 0) 
	break; 

      if(strcmp(keyword, ":hierarchy") == 0) 
      { 
	MOV_BONES_IN_ASF_FILE -= 1; 
	NUM_BONES_IN_ASF_FILE -= 1; 
	done=true; 
	break; 
      }

      //id of bone
      if(strcmp(keyword, "id") == 0)
        m_pBoneList[i].idx = NUM_BONES_IN_ASF_FILE-1;

      //name of the bone
      if(strcmp(keyword, "name") == 0) 
      {
        sscanf(str, "%s %s", keyword, part);
        sscanf(str, "%s %s", keyword, m_pBoneList[i].name);
      }

      //this line describes the bone's direction vector in global coordinates
      //it will later be converted to local coorinate system
      if(strcmp(keyword, "direction") == 0)  
        sscanf(str, "%s %lf %lf %lf", keyword, &m_pBoneList[i].dir[0], &m_pBoneList[i].dir[1], &m_pBoneList[i].dir[2]);

      //length of the bone
      if(strcmp(keyword, "length") == 0)  
        sscanf(str, "%s %lf", keyword, &length);

      //this line describes the orientation of bone's local coordinate 
      //system relative to the world coordinate system
      if(strcmp(keyword, "axis") == 0)      
        sscanf(str, "%s %lf %lf %lf", keyword, &m_pBoneList[i].axis_x, &m_pBoneList[i].axis_y, &m_pBoneList[i].axis_z);

      // this line describes the bone's dof 
      if(strcmp(keyword, "dof") == 0)       
      {
        token=strtok(str, " "); 
        m_pBoneList[i].dof=0;
        while(token != NULL)      
        {
          int tdof = m_pBoneList[i].dof;

          if(strcmp(token, "rx") == 0) { m_pBoneList[i].dofrx = 1; m_pBoneList[i].dofo[tdof] = 1; }
          else if(strcmp(token, "ry") == 0) { m_pBoneList[i].dofry = 1; m_pBoneList[i].dofo[tdof] = 2; }
          else if(strcmp(token, "rz") == 0) { m_pBoneList[i].dofrz = 1; m_pBoneList[i].dofo[tdof] = 3; }
          else if(strcmp(token, "tx") == 0) { m_pBoneList[i].doftx = 1; m_pBoneList[i].dofo[tdof] = 4; }
          else if(strcmp(token, "ty") == 0) { m_pBoneList[i].dofty = 1; m_pBoneList[i].dofo[tdof] = 5; }
          else if(strcmp(token, "tz") == 0) { m_pBoneList[i].doftz = 1; m_pBoneList[i].dofo[tdof] = 6; }
          else if(strcmp(token, "l") == 0)  { m_pBoneList[i].doftl = 1; m_pBoneList[i].dofo[tdof] = 7; }
          else if(strcmp(token, "dof") == 0) { goto end; }
          else { printf("UNKNOWN %s\n",token); }

          m_pBoneList[i].dof++;
          m_pBoneList[i].dofo[m_pBoneList[i].dof] = 0;
          end:
            token=strtok(NULL, " ");
        }
        printf("Bone %d DOF: ",i);
        for (int x = 0; (x < 7) && (m_pBoneList[i].dofo[x] != 0); x++) 
	  printf("%d ",m_pBoneList[i].dofo[x]);
        printf("\n");
      }
    }

    //store all the info we read from the file into the data structure
    //		m_pBoneList[i].idx = name2idx(part);
    if ((!m_pBoneList[i].dofrx) && (!m_pBoneList[i].dofry) && (!m_pBoneList[i].dofrz))
      MOV_BONES_IN_ASF_FILE -= 1;
    m_pBoneList[i].length = length * scale;
  }
  printf("READ %d\n",NUM_BONES_IN_ASF_FILE);

  //
  //read and build the hierarchy of the skeleton
  //
  char *part_name;
  int j, parent = 0;

  //find "hierarchy" string in the ASF file
  /*	while(1)
  {
  is.getline(str, 2048);	sscanf(str, "%s", keyword);
  if(strcmp(keyword, ":hierarchy") == 0)	
  break;
  } */

  //skip "begin" line
  is.getline(str, 2048);
  removeCR(str);

  //Assign parent/child relationship to the bones
  while(1)
  {
    //read next line
    is.getline(str, 2048);	
    removeCR(str);

    sscanf(str, "%s", keyword);

    //check if we are done
    if(strcmp(keyword, "end") == 0)   
      break;
    else
    {
      //parse this line, it contains parent followed by children
      part_name=strtok(str, " ");
      j=0;
      while(part_name != NULL)
      {
        if(j==0) 
          parent=name2idx(part_name);
        else 
          setChildrenAndSibling(parent, &m_pBoneList[name2idx(part_name)]);
        part_name=strtok(NULL, " ");
        j++;
      }
    }
  }

  is.close();

  return 0;
}


/*
This recursive function traverces skeleton hierarchy 
and returns a pointer to the bone with index - bIndex
ptr should be a pointer to the root node 
when this function first called
*/
Bone* CMUSkeleton::getBone(Bone *ptr, int bIndex)
{
  static Bone *theptr;
  if(ptr==NULL) 
    return(NULL);
  else if(ptr->idx == bIndex)
  {
    theptr=ptr;
    return(theptr);
  }
  else
  { 
    getBone(ptr->child, bIndex);
    getBone(ptr->sibling, bIndex);
    return(theptr);
  }
}

/*
This function sets sibling or child for parent bone
If parent bone does not have a child, 
then pChild is set as parent's child
else pChild is set as a sibling of parents already existing child
*/
int CMUSkeleton::setChildrenAndSibling(int parent, Bone *pChild)
{
  Bone *pParent;  

  //Get pointer to root bone
  pParent = getBone(m_pRootBone, parent);

  if(pParent==NULL)
  {
    printf("inbord bone is undefined\n"); 
    return(0);
  }
  else
  {
    //if pParent bone does not have a child
    //set pChild as parent bone child
    if(pParent->child == NULL)   
    {
      pParent->child = pChild;
    }
    else
    { 
      //if pParent bone already has a child 
      //set pChils as pParent bone's child sibling
      pParent=pParent->child;              
      while(pParent->sibling != NULL) 
        pParent = pParent->sibling;            

      pParent->sibling = pChild;
    }
    return(1);
  }
}

/* 
Return the pointer to the root bone
*/	
Bone * CMUSkeleton::getRoot()
{
  return(m_pRootBone);
}

Bone* CMUSkeleton::getBone(int idx)
{
	return &(m_pBoneList[idx]);
}


/***************************************************************************************
Compute relative orientation and translation between the 
parent and child bones. That is, represent the orientation 
matrix and translation vector in the local coordinate of parent body 
*****************************************************************************************/





/******************************************************************************
Interface functions to set the pose of the skeleton 
******************************************************************************/


//Set the aspect ratio of each bone 
void CMUSkeleton::set_bone_shape(Bone *bone)
{
	int root = CMUSkeleton::getRootIndex();
  bone[root].aspx=1;          
  bone[root].aspy=1;
  printf("READ %d\n",numBonesInSkel(bone[0]));
  printf("MOV %d\n",movBonesInSkel(bone[0]));
  int numbones = numBonesInSkel(bone[0]);
  for(int j=1;j<numbones;j++)
  {
    bone[j].aspx=0.25;   
    bone[j].aspy=0.25;
  }
}

// Constructor 
CMUSkeleton::CMUSkeleton(const char *asf_filename, double scale)
{
  sscanf("root","%s",m_pBoneList[0].name);
  NUM_BONES_IN_ASF_FILE = 1;
  MOV_BONES_IN_ASF_FILE = 1;
  m_pBoneList[0].dofo[0] = 4;
  m_pBoneList[0].dofo[1] = 5;
  m_pBoneList[0].dofo[2] = 6;
  m_pBoneList[0].dofo[3] = 1;
  m_pBoneList[0].dofo[4] = 2;
  m_pBoneList[0].dofo[5] = 3;
  m_pBoneList[0].dofo[6] = 0;
  //Initialization
  m_pBoneList[0].idx = getRootIndex();   // root of hierarchy
  m_pRootBone = &m_pBoneList[0];
  m_pBoneList[0].sibling = NULL;
  m_pBoneList[0].child = NULL; 
  m_pBoneList[0].dir[0] = 0; m_pBoneList[0].dir[1] = 0.; m_pBoneList[0].dir[2] = 0.;
  m_pBoneList[0].axis_x = 0; m_pBoneList[0].axis_y = 0.; m_pBoneList[0].axis_z = 0.;
  m_pBoneList[0].length = 0.05;
  m_pBoneList[0].dof = 6;
  m_pBoneList[0].dofrx = m_pBoneList[0].dofry = m_pBoneList[0].dofrz = 1;
  m_pBoneList[0].doftx = m_pBoneList[0].dofty = m_pBoneList[0].doftz = 1;
  m_pBoneList[0].doftl = 0;
  m_RootPos[0] = m_RootPos[1]=m_RootPos[2]=0;
  //	m_NumDOFs=6;
  tx = ty = tz = rx = ry = rz = 0.0;
  // build hierarchy and read in each bone's DOF information
  int code = readASFfile(asf_filename, scale);  
  if (code != 0)
    throw 1;

  //transform the direction vector for each bone from the world coordinate system 
  //to it's local coordinate system
  //RotateBoneDirToLocalCoordSystem();

  //Calculate rotation from each bone local coordinate system to the coordinate system of its parent
  //store it in rot_parent_current variable for each bone
  //ComputeRotationToParentCoordSystem(m_pRootBone);

  //Set the aspect ratio of each bone 
  set_bone_shape(m_pRootBone);
}

CMUSkeleton::~CMUSkeleton()
{
}

void CMUSkeleton::GetRootPosGlobal(double rootPosGlobal[3])
{
  rootPosGlobal[0] = m_RootPos[0];
  rootPosGlobal[1] = m_RootPos[1];
  rootPosGlobal[2] = m_RootPos[2];
}

void CMUSkeleton::GetTranslation(double translation[3])
{
  translation[0] = tx;
  translation[1] = ty;
  translation[2] = tz;
}

void CMUSkeleton::GetRotationAngle(double rotationAngle[3])
{
  rotationAngle[0] = rx;
  rotationAngle[1] = ry;
  rotationAngle[2] = rz;
}


