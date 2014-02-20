vector<int> evaluatedSet;
vector<pair <int, float> > expandedSet;
vector<float> h_value;
vector<float> g_value;
vector<float> f_value;
vector<int> prev;
vector<int> pathNodes;
PxVec3 hDiff;
int curr, currPos;
float minF;
currentPlanStep = 0;
for (int i=0;i<g.graphPoints.size();i++)
{
    hDiff= g.graphPoints[i]-(g.graphPoints[goal]);
    h_value.push_back(hDiff.magnitude());
    g_value.push_back(0.0);
    f_value.push_back(0.0);
    prev.push_back(-1);
}
f_value[start]=g_value[start]+h_value[start];
expandedSet.push_back(make_pair(start,f_value[start]));
while(!expandedSet.empty())
{
    minF = expandedSet[0].second;
    curr = expandedSet[0].first;
    currPos=0;
    for (int i=1;i<expandedSet.size();i++)
        if (expandedSet[i].second < minF)
        {
            minF = expandedSet[i].second;
            curr = expandedSet[i].first;
            currPos = i;
        }
    if (curr == goal)
    {
        havePlanned = true;
        while(curr!=-1)
        {
            pathNodes.push_back(curr);
            curr=prev[curr];
        }
        for (int i=pathNodes.size()-1;i>=0;i--)
            plan.push_back(g.graphPoints[pathNodes[i]]);
            break;
    }
    else
    {
        expandedSet.erase(expandedSet.begin()+currPos);
        evaluatedSet.push_back(curr);
        for (int i=0; i<g.graphEdges.size(); i++)
        {
            if (g.graphEdges[i].first == curr)
            {
                int adjacent=g.graphEdges[i].second;
                float temp_g,temp_f;
                bool isEvalSet=false;
                bool isExpSet=false;
                PxVec3 dist=g.graphPoints[curr]-(g.graphPoints[adjacent]);
                temp_g=g_value[curr]+dist.magnitude();
                temp_f=temp_g+h_value[adjacent];
                for (int j=0;j<evaluatedSet.size();j++)
                    if(evaluatedSet[j] == adjacent)
                        isEvalSet = true;
                        if (isEvalSet==true && temp_f >= f_value[adjacent])
                            continue;
                for (int j=0;j<expandedSet.size();j++)
                    if(expandedSet[j].first == adjacent)
                        isExpSet = true;
                        if(isExpSet==false || temp_f < f_value[adjacent])
                        {
                            for(int j=0;j<expandedSet.size();j++)
                                if (expandedSet[j].first==adjacent)
                                {
                                    expandedSet[j].second=temp_f;
                                    break;
                                }
                            prev[adjacent]=curr;
                            g_value[adjacent]=temp_g;
                            f_value[adjacent]=temp_f;
                            if (!isExpSet)
                                expandedSet.push_back(make_pair(adjacent, f_value[adjacent]));
                                }
            }
        }
    }
