// ----------------------------------------------------------------------------
//
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
// Simple soccer game by Michael Holm, IO Interactive A/S
//
// I made this to learn opensteer, and it took me four hours. The players will
// hunt the ball with no team spirit, each player acts on his own accord.
//
// I challenge the reader to change the behavour of one of the teams, to beat my
// team. It should not be too hard. If you make a better team, please share the
// source code so others get the chance to create a team that'll beat yours :)
//
// You are free to use this code for whatever you please.
//
// (contributed on July 9, 2003)
//
// ----------------------------------------------------------------------------

/**
 * \file /Ely/include/Support/OpenSteerLocal/PlugIn_Soccer.h
 *
 * \date 2014-03-31 
 * \author consultit
 */

#ifndef PLUGIN_SOCCER_H_
#define PLUGIN_SOCCER_H_

#include <iomanip>
#include <sstream>
#include <OpenSteer/Draw.h>
#include <OpenSteer/Color.h>
#include <OpenSteer/UnusedParameter.h>
#include <OpenSteer/PlugIn.h>
#include "common.h"

namespace ossup
{

using namespace OpenSteer;

// ----------------------------------------------------------------------------

// a box object for the field and the goals.
class AABBox
{
public:
	AABBox(const Vec3 &min, const Vec3& max) :
			m_min(min), m_max(max)
	{
		//reorder min/max
		float tmp;
		if (m_min.x > m_max.x)
		{
			tmp = m_min.x;
			m_min.x = m_max.x;
			m_max.x = tmp;
		}
		if (m_min.y > m_max.y)
		{
			tmp = m_min.y;
			m_min.y = m_max.y;
			m_max.y = tmp;
		}
		if (m_min.z > m_max.z)
		{
			tmp = m_min.z;
			m_min.z = m_max.z;
			m_max.z = tmp;
		}
		m_mid_y = (m_min.y + m_max.y) / 2.0;
	}
///	AABBox(Vec3 min, Vec3 max) :
///			m_min(min), m_max(max)
///	{
///		m_mid_y = (m_min.y + m_max.y) / 2.0;
///	}
	bool InsideX(const Vec3 p)
	{
		if (p.x < m_min.x || p.x > m_max.x)
			return false;
		return true;
	}
	bool InsideZ(const Vec3 p)
	{
		if (p.z < m_min.z || p.z > m_max.z)
			return false;
		return true;
	}
#ifdef OS_DEBUG
	void draw()
	{
		Vec3 b, c;
		b = Vec3(m_min.x, m_mid_y, m_max.z);
		c = Vec3(m_max.x, m_mid_y, m_min.z);
		Color color(0.2f, 0.2f, 0.0f);
		drawLineAlpha(m_min, b, color, 1.0f);
		drawLineAlpha(b, m_max, color, 1.0f);
		drawLineAlpha(m_max, c, color, 1.0f);
		drawLineAlpha(c, m_min, color, 1.0f);
	}
#endif

	Vec3 getMin()
	{
		return m_min;
	}
	Vec3 getMax()
	{
		return m_max;
	}
private:
	Vec3 m_min;
	Vec3 m_max;
	float m_mid_y;
};

// The ball object
template<typename Entity>
class Ball: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a ball: an STL vector of Ball pointers
	typedef typename std::vector<Ball<Entity>*> groupType;

///	Ball(AABBox *bbox) :
///			m_bbox(bbox)
	Ball():m_bbox(NULL)
	{
		//set default home
		m_home = this->m_start;
		reset();
	}

	// reset state
	void reset(void)
	{
		SimpleVehicle::reset(); // reset the vehicle
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();

///		setSpeed(0.0f);         // speed along Forward direction.
///		setMaxForce(9.0f);      // steering force is clipped to this magnitude
///		setMaxSpeed(9.0f);         // velocity is clipped to this magnitude
		this->setPosition(m_home);

#ifdef OS_DEBUG
		this->clearTrailHistory();    // prevent long streaks due to teleportation
		this->setTrailParameters(100, 6000);
#endif
	}

	// per frame simulation update
	void update(const float currentTime, const float elapsedTime)
	{
		this->applyBrakingForce(1.5f, elapsedTime);
		this->applySteeringForce(this->velocity(), elapsedTime);
		// are we now outside the field?
		if (!m_bbox->InsideX(this->position()))
		{
			Vec3 d = this->velocity();
			this->regenerateOrthonormalBasis(Vec3(-d.x, d.y, d.z));
			this->applySteeringForce(this->velocity(), elapsedTime);
		}
		if (!m_bbox->InsideZ(this->position()))
		{
			Vec3 d = this->velocity();
			this->regenerateOrthonormalBasis(Vec3(d.x, d.y, -d.z));
			this->applySteeringForce(this->velocity(), elapsedTime);
		}

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		this->recordTrailVertex(currentTime, this->position());
#endif
	}

	virtual void kick(Vec3 dir, const float elapsedTime)
	{
		OPENSTEER_UNUSED_PARAMETER(elapsedTime);

		this->setSpeed(dir.length());
		this->regenerateOrthonormalBasis(dir);
	}

#ifdef OS_DEBUG
	// draw this character/vehicle into the scene
	void draw(void)
	{
		drawBasic2dCircularVehicle(*this, Color(0.0f, 1.0f, 0.0f));
		this->drawTrail();
	}
#endif

	AABBox *m_bbox;
	Vec3 m_home;
};

//Ball externally updated.
template<typename Entity>
class ExternalBall: public Ball<Entity>
{
public:
	void update(const float currentTime, const float elapsedTime)
	{
		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		this->recordTrailVertex(currentTime, this->position());
#endif
	}
};

template<typename Entity>
class Player: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a player: an STL vector of Player pointers
	typedef typename std::vector<Player<Entity>*> groupType;

	// constructor
///	Player(std::vector<Player*> others, std::vector<Player*> allplayers,
///			Ball* ball, bool isTeamA, int id) :
///			m_others(others), m_AllPlayers(allplayers), m_Ball(ball), b_ImTeamA(
///					isTeamA), m_MyID(id)
	Player()
	{
		m_Ball = NULL;
		b_ImTeamA = true;
		m_TeamAssigned = false;
		//set default home
		m_home = this->m_start;
		reset();
	}

	// reset state
	void reset(void)
	{
		SimpleVehicle::reset(); // reset the vehicle
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();

///		setSpeed(0.0f);         // speed along Forward direction.
///		setMaxForce(3000.7f);     // steering force is clipped to this magnitude
///		setMaxSpeed(10);         // velocity is clipped to this magnitude

		// Place me on my part of the field, looking at oponnents goal
///		this->setPosition(b_ImTeamA ? frandom01() * 20 : -frandom01() * 20, 0,
///				(frandom01() - 0.5f) * 20);
///		if (m_MyID < 9)
///		{
///			if (b_ImTeamA)
///				this->setPosition(playerPosition[m_MyID]);
///			else
///				this->setPosition(
///						Vec3(-playerPosition[m_MyID].x,
///								playerPosition[m_MyID].y,
///								playerPosition[m_MyID].z));
///		}
///		m_home = this->position();

#ifdef OS_DEBUG
		this->clearTrailHistory();    // prevent long streaks due to teleportation
		this->setTrailParameters(10, 60);
#endif
	}

	// per frame simulation update
	// (parameter names commented out to prevent compiler warning from "-W")
	void update(const float currentTime, const float elapsedTime)
	{
		if ((not m_Ball) or (not m_TeamAssigned))
		{
			// otherwise consider avoiding collisions with others
			Vec3 collisionAvoidance = this->steerToAvoidNeighbors(1,
					(AVGroup&) *m_AllPlayers);
			if (collisionAvoidance != Vec3::zero)
				this->applySteeringForce(collisionAvoidance, elapsedTime);
			else
			{
				//reach your home until there is a ball
				Vec3 seekTarget = this->xxxsteerForSeek(m_home);
				Vec3 seekHome = this->xxxsteerForSeek(m_home);
				this->applySteeringForce(seekTarget + seekHome, elapsedTime);
			}
		}
		else
		{
			// if I hit the ball, kick it.
			const float distToBall = Vec3::distance(this->position(),
					m_Ball->position());
			const float sumOfRadii = this->radius() + m_Ball->radius();
			if (distToBall < sumOfRadii)
				m_Ball->kick((m_Ball->position() - this->position()) * 50,
						elapsedTime);

			// otherwise consider avoiding collisions with others
			Vec3 collisionAvoidance = this->steerToAvoidNeighbors(1,
					(AVGroup&) *m_AllPlayers);
			if (collisionAvoidance != Vec3::zero)
				this->applySteeringForce(collisionAvoidance, elapsedTime);
			else
			{
				float distHomeToBall = Vec3::distance(m_home,
						m_Ball->position());
				if (distHomeToBall < m_distHomeToBall)
				{
					// go for ball if I'm on the 'right' side of the ball
					if (b_ImTeamA ?
							this->position().x > m_Ball->position().x :
							this->position().x < m_Ball->position().x)
					{
						Vec3 seekTarget = this->xxxsteerForSeek(
								m_Ball->position());
						this->applySteeringForce(seekTarget, elapsedTime);
					}
					else
					{
///					if (distHomeToBall < m_distHomeToBall)
///					{
						float Z =
								m_Ball->position().z - this->position().z > 0 ?
										-1.0f : 1.0f;
						Vec3 behindBall = m_Ball->position()
								+ (b_ImTeamA ?
										Vec3(2.0f, 0.0f, Z) :
										Vec3(-2.0f, 0.0f, Z));
						Vec3 behindBallForce = this->xxxsteerForSeek(
								behindBall);
#ifdef OS_DEBUG
						this->annotationLine(this->position(), behindBall,
								Color(0.0f, 1.0f, 0.0f));
#endif
						Vec3 evadeTarget = this->xxxsteerForFlee(
								m_Ball->position());
						this->applySteeringForce(
								behindBallForce * 10.0f + evadeTarget,
								elapsedTime);
///					}
					}
				}
				else	// Go home
				{
					Vec3 seekTarget = this->xxxsteerForSeek(m_home);
					Vec3 seekHome = this->xxxsteerForSeek(m_home);
					this->applySteeringForce(seekTarget + seekHome,
							elapsedTime);
				}
			}
		}

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);
	}

#ifdef OS_DEBUG
	// draw this character/vehicle into the scene
	void draw(void)
	{
		drawBasic2dCircularVehicle(*this,
				b_ImTeamA ? Color(1.0f, 0.0f, 0.0f) : Color(0.0f, 0.0f, 1.0f));
		this->drawTrail();
	}
#endif
	// per-instance reference to its group
///	const std::vector<Player*> m_others;
	std::vector<Player*>* m_AllPlayers;
	Ball<Entity>* m_Ball;
	bool b_ImTeamA;
	bool m_TeamAssigned;
///	int m_MyID;
	Vec3 m_home;
	float m_distHomeToBall;
};

//Player externally updated.
template<typename Entity>
class ExternalPlayer: public Player<Entity>
{
public:
	void update(const float currentTime, const float elapsedTime)
	{

		// if I hit the ball, kick it.
		const float distToBall = Vec3::distance(this->position(), this->m_Ball->position());
		const float sumOfRadii = this->radius() + this->m_Ball->radius();
		if (distToBall < sumOfRadii)
			this->m_Ball->kick((this->m_Ball->position() - this->position()) * 50, elapsedTime);

		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		//annotation
		Vec3 collisionAvoidance = this->steerToAvoidNeighbors(1,
				(AVGroup&) this->m_AllPlayers);
		if (collisionAvoidance == Vec3::zero)
		{
			float distHomeToBall = Vec3::distance(this->m_start,
					this->m_Ball->position());
			if (distHomeToBall < 12.0f)
			{
				// go for ball if I'm on the 'right' side of the ball
				if (not (
						this->b_ImTeamA ?
								this->position().x > this->m_Ball->position().x :
								this->position().x < this->m_Ball->position().x))
				{
					float Z =
							this->m_Ball->position().z - this->position().z > 0 ?
									-1.0f : 1.0f;
					Vec3 behindBall = this->m_Ball->position()
							+ (this->b_ImTeamA ?
									Vec3(2.0f, 0.0f, Z) : Vec3(-2.0f, 0.0f, Z));
					this->annotationLine(this->position(), behindBall,
							Color(0.0f, 1.0f, 0.0f));
				}
			}
		}
#endif
	}
};

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo
template<typename Entity>
class MicTestPlugIn: public PlugIn
{
public:

	const char* name(void)
	{
		return "Michael's Simple Soccer";
	}

	// float selectionOrderSortKey (void) {return 0.06f;}

	// bool requestInitialSelection() { return true;}

	// be more "nice" to avoid a compiler warning
	virtual ~MicTestPlugIn()
	{
	}

	void open(void)
	{
		// Make a field
		m_bbox = new AABBox(Vec3(-20, 0, -10), Vec3(20, 0, 10));
		// Red goal
		m_TeamAGoal = new AABBox(Vec3(-21, 0, -7), Vec3(-19, 0, 7));
		// Blue Goal
		m_TeamBGoal = new AABBox(Vec3(19, 0, -7), Vec3(21, 0, 7));
		// Make a ball
///		m_Ball = new Ball(m_bbox);
		m_Ball = NULL;
///		m_AllVehicles.push_back(m_Ball);
///		// Build team A
///		m_PlayerCountA = 8;
///		for (unsigned int i = 0; i < m_PlayerCountA; i++)
///		{
///			Player *pMicTest = new Player(TeamA, m_AllPlayers, m_Ball, true, i);
///			selectedVehicle = pMicTest;
///			TeamA.push_back(pMicTest);
///			m_AllPlayers.push_back(pMicTest);
///			m_AllVehicles.push_back(pMicTest);
///		}
///		// Build Team B
///		m_PlayerCountB = 8;
///		for (unsigned int i = 0; i < m_PlayerCountB; i++)
///		{
///			Player *pMicTest = new Player(TeamB, m_AllPlayers, m_Ball, false,
///					i);
///			selectedVehicle = pMicTest;
///			TeamB.push_back(pMicTest);
///			m_AllPlayers.push_back(pMicTest);
///			m_AllVehicles.push_back(pMicTest);
///		}

		m_redScore = 0;
		m_blueScore = 0;
	}

	void update(const float currentTime, const float elapsedTime)
	{
		// update simulation of test vehicle
///		for (unsigned int i = 0; i < m_PlayerCountA; i++)
///			TeamA[i]->update(currentTime, elapsedTime);
///		for (unsigned int i = 0; i < m_PlayerCountB; i++)
///			TeamB[i]->update(currentTime, elapsedTime);
///		m_Ball->update(currentTime, elapsedTime);
		AVIterator iter;
		for (iter = m_AllVehicles.begin(); iter != m_AllVehicles.end(); ++iter)
		{
			(*iter)->update(currentTime, elapsedTime);
		}

		if (not m_Ball)
		{
			return;
		}

		if (m_TeamAGoal->InsideX(m_Ball->position())
				&& m_TeamAGoal->InsideZ(m_Ball->position()))
		{
			m_Ball->reset();	// Ball in blue teams goal, red scores
			m_redScore++;
		}
		if (m_TeamBGoal->InsideX(m_Ball->position())
				&& m_TeamBGoal->InsideZ(m_Ball->position()))
		{
			m_Ball->reset();	// Ball in red teams goal, blue scores
			m_blueScore++;
		}

	}

	void redraw(const float currentTime, const float elapsedTime)
	{
#ifdef OS_DEBUG
		// draw test vehicle
///		for (unsigned int i = 0; i < m_PlayerCountA; i++)
///			TeamA[i]->draw();
///		for (unsigned int i = 0; i < m_PlayerCountB; i++)
///			TeamB[i]->draw();
		iterator iter;
		for (iter = m_AllPlayers.begin(); iter != m_AllPlayers.end(); ++iter)
		{
			(*iter)->draw();
		}
		if (m_Ball)
		{
			m_Ball->draw();
		}

		///FIXME: delegated to external plugin initialization
///		drawPlayField();

		{
			std::ostringstream annote;
			annote << "Red: " << m_redScore;
/////			draw2dTextAt3dLocation(annote, Vec3(23, 0, 0),
/////					Color(1.0f, 0.7f, 0.7f), drawGetWindowWidth(),
/////					drawGetWindowHeight());
			draw2dTextAt3dLocation(annote, Vec3(23, 0, 0),
					Color(1.0f, 0.7f, 0.7f), 0.0, 0.0);
		}
		{
			std::ostringstream annote;
			annote << "Blue: " << m_blueScore;
/////			draw2dTextAt3dLocation(annote, Vec3(-23, 0, 0),
/////					Color(0.7f, 0.7f, 1.0f), drawGetWindowWidth(),
/////					drawGetWindowHeight());
			draw2dTextAt3dLocation(annote, Vec3(-23, 0, 0),
					Color(0.7f, 0.7f, 1.0f), 0.0, 0.0);
		}

		// textual annotation (following the test vehicle's screen position)
		if (0)
			for (unsigned int i = 0; i < m_AllPlayers.size(); i++)
			{
				std::ostringstream annote;
				annote << std::setprecision(2)
						<< std::setiosflags(std::ios::fixed);
				annote << "      speed: " << m_AllPlayers[i]->speed() << "ID:" << i
						<< std::ends;
/////				draw2dTextAt3dLocation(annote, TeamA[i]->position(), gRed,
/////						drawGetWindowWidth(), drawGetWindowHeight());
/////				draw2dTextAt3dLocation(*"start", Vec3::zero, gGreen,
/////						drawGetWindowWidth(), drawGetWindowHeight());
				draw2dTextAt3dLocation(annote, m_AllPlayers[i]->position(), gRed, 0.0,
						0.0);
				draw2dTextAt3dLocation(*"start", Vec3::zero, gGreen, 0.0, 0.0);
			}
#endif
	}

#ifdef OS_DEBUG
	void drawSoccerField()
	{
		gDrawer3d->setTwoSided(true);
		m_bbox->draw();
		Vec3 c = (m_bbox->getMax() + m_bbox->getMin()) / 2.0;
		Vec3 d = (m_bbox->getMax() - m_bbox->getMin()) / 2.0;
		//draw a middle line on xz plane
		Color color(0.2f, 0.2f, 0.0f);
		drawLineAlpha(c + Vec3(0, 0, d.z), c - Vec3(0, 0, d.z), color, 1.0f);
		//draw a middle circle on xz plane
		drawCircleOrDisk(d.x / 4.0, Vec3(1, 0, 0), c, color, 32, false, false);
		gDrawer3d->setTwoSided(false);
		m_TeamAGoal->draw();
		m_TeamBGoal->draw();
	}
#endif

	void close(void)
	{
///		for (unsigned int i = 0; i < m_PlayerCountA; i++)
///			delete TeamA[i];
///		TeamA.clear();
///		for (unsigned int i = 0; i < m_PlayerCountB; i++)
///			delete TeamB[i];
///		TeamB.clear();
///		delete m_Ball;
///		m_AllPlayers.clear();
///		m_AllVehicles.clear();
		delete m_bbox;
		delete m_TeamAGoal;
		delete m_TeamBGoal;
	}

	void reset(void)
	{
		// reset vehicle
///		for (unsigned int i = 0; i < m_PlayerCountA; i++)
///			TeamA[i]->reset();
///		for (unsigned int i = 0; i < m_PlayerCountB; i++)
///			TeamB[i]->reset();
		iterator iter;
		for (iter = m_AllPlayers.begin(); iter != m_AllPlayers.end(); ++iter)
		{
			(*iter)->reset();
		}
		if (m_Ball)
		{
			m_Ball->reset();
		}
	}

	virtual bool addVehicle(AbstractVehicle* vehicle)
	{
		if(not PlugInAddOnMixin<OpenSteer::PlugIn>::addVehicle(vehicle))
		{
			return false;
		}
		//check if this is a Ball
		Ball<Entity>* ballTmp =
				dynamic_cast<Ball<Entity>*>(vehicle);
		if (ballTmp)
		{
			// set the plugin's ball: the last added one
			m_Ball = ballTmp;
			// set the ball's AABB
			m_Ball->m_bbox = m_bbox;
			// update the ball home: the field center
			m_Ball->m_home = (m_bbox->getMin() + m_bbox->getMax()) / 2.0;
			m_Ball->reset();
			//update each player's ball
			setAllPlayersBall();
			//that's all
			return true;
		}
		//or if this is a Player
		Player<Entity>* playerTmp =
			dynamic_cast<Player<Entity>*>(vehicle);
		if (playerTmp)
		{
			// add player to all players' repo
			m_AllPlayers.push_back(playerTmp);
			// set the player's all player repo
			playerTmp->m_AllPlayers = &m_AllPlayers;
			// set the player's ball
			playerTmp->m_Ball = m_Ball;
			//that's all
			return true;
		}
		//roll back addition
		PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle);
		//
		return false;
	}

	virtual bool removeVehicle(OpenSteer::AbstractVehicle* vehicle)
	{
		if(not PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle))
		{
			return false;
		}
		//check if this is a Player
		Player<Entity>* playerTmp =
			dynamic_cast<Player<Entity>*>(vehicle);
		if (playerTmp)
		{
			//remove it also from all players' repo
			iterator iter;
			for (iter = m_AllPlayers.begin(); iter != m_AllPlayers.end();
					++iter)
			{
				if (*iter == playerTmp)
				{
					m_AllPlayers.erase(iter);
					break;
				}
			}

			//remove it also from its team, if already added
			removePlayerFromTeam(playerTmp);
		}
		//check if this is the current ball
		if (vehicle == m_Ball)
		{
			m_Ball = NULL;
			// find a new ball (if any): the first found or NULL
			AVIterator iter;
			for (iter = m_AllVehicles.begin(); iter != m_AllVehicles.end(); ++iter)
			{
				m_Ball = dynamic_cast<Ball<Entity>*>(*iter);
				if (m_Ball)
				{
					break;
				}
			}
			//update each player's ball
			setAllPlayersBall();
		}
		//
		return true;
	}

	void setAllPlayersBall()
	{
		iterator iter;
		for (iter = m_AllPlayers.begin(); iter != m_AllPlayers.end(); ++iter)
		{
			(*iter)->m_Ball = m_Ball;
		}
	}

	void addPlayerToTeam(Player<Entity>* player, bool teamA)
	{
		if(player->m_TeamAssigned == true)
		{
			return;
		}
		//add player to its team, if not already added
		typename Player<Entity>::groupType* team = (
				teamA ? &TeamA : &TeamB);
		iterator iter;
		for (iter = (*team).begin(); iter != (*team).end(); ++iter)
		{
			if (*iter == player)
			{
				break;
			}
		}
		if (iter == (*team).end())
		{
			(*team).push_back(player);
			// update the team players' homes
			updatePlayersHome(team);
			//
			player->b_ImTeamA = teamA;
			player->m_TeamAssigned = true;
		}
	}

	void removePlayerFromTeam(Player<Entity>* player)
	{
		if(player->m_TeamAssigned == false)
		{
			return;
		}
		//remove it also from its team, if already added
		typename Player<Entity>::groupType* team = (
				player->b_ImTeamA ? &TeamA : &TeamB);
		iterator iter;
		for (iter = (*team).begin(); iter != (*team).end(); ++iter)
		{
			if (*iter == player)
			{
				break;
			}
		}
		if (iter != (*team).end())
		{
			(*team).erase(iter);
			// update the remaining team players' homes
			updatePlayersHome(team);
			//
			player->m_TeamAssigned = false;
		}
	}

	void updatePlayersHome(typename Player<Entity>::groupType* team)
	{
		unsigned int numPlayers = (*team).size();
		if (numPlayers == 0)
		{
			return;
		}
		//get half field dims
		float dimX = abs(m_bbox->getMax().x - m_bbox->getMin().x) / 2.0;
		float dimZ = abs(m_bbox->getMax().z - m_bbox->getMin().z);
		//compute distHomeToBall: diagonal * 0.424264069
		float distHomeToBall = sqrt(dimX * dimX + dimZ * dimZ) * 0.424264069;
		//set k factor = MaxDim / minDim
		float k = (dimX >= dimZ ? dimX / dimZ : dimZ / dimX);
		//get subdivisions for the shorter dim (float)
		float subv = sqrt((float) numPlayers / k);
		//set min and max subdivisions (int)
		int subvMinI = ((subv - (int) subv) > 0.0 ? 1 : 0) + (int) subv;
		float subvMax = (float) numPlayers / (float) subvMinI;
		int subvMaxI = ((subvMax - (int) subvMax) > 0.0 ? 1 : 0)
				+ (int) subvMax;
		//set x,z data
		int numPlayerX, numPlayerZ;
		float dX, dZ;
		if (dimX >= dimZ)
		{
			numPlayerX = subvMaxI;
			numPlayerZ = subvMinI;
		}
		else
		{
			numPlayerX = subvMinI;
			numPlayerZ = subvMaxI;
		}
		dX = dimX / (float) (numPlayerX + 1);
		dZ = dimZ / (float) (numPlayerZ + 1);
		//invert dX direction for team B
		if (team == &TeamB)
		{
			dX = -dX;
		}
		//get field low center
		Vec3 lowCenter =
				(m_bbox->getMin() + m_bbox->getMax() - Vec3(0, 0, dimZ)) / 2.0;
		//set players' homes
		iterator iter = (*team).begin();
		int XI = (int) (frandom01() * (numPlayerX - 1));
		for (int xi = 0; xi < numPlayerX; ++xi)
		{
			//evenly distribute along last column
			if (xi == (numPlayerX - 1))
			{
				//recompute last players
				numPlayerZ = numPlayers - numPlayerZ * xi;
				dZ = dimZ / (float) (numPlayerZ + 1);
			}

			int ZI = (int) (frandom01() * (numPlayerZ - 1));
			for (int zi = 0; zi < numPlayerZ; ++zi)
			{
				//update m_home
				(*iter)->m_home = lowCenter
						+ Vec3((XI + 1) * dX + frandom01() * (*iter)->radius(), 0,
								(ZI + 1) * dZ + frandom01() * (*iter)->radius());
				//update also m_distHomeToBall
				(*iter)->m_distHomeToBall = distHomeToBall;
				//go on
				++iter;
				if (iter == (*team).end())
				{
					break;
				}
				ZI = (ZI + 1) % numPlayerZ;
			}
			//
			if (iter == (*team).end())
			{
				break;
			}
			XI = (XI + 1) % numPlayerX;
		}
	}

	void setSoccerField(const Vec3& min, const Vec3& max)
	{
		//delete old boxes
		delete m_bbox;
		delete m_TeamAGoal;
		delete m_TeamBGoal;
		//add new boxes, with minimum soccer field dims: 40x20
		float xDim = abs(max.x - min.x), zDim = abs(max.z - min.z);
		if (xDim < 40)
		{
			xDim = 40;
		}
		if (zDim < 20)
		{
			zDim = 20;
		}
		//new min/max: middle point -/+ half dim
		Vec3 middle = (max + min) / 2.0, halfDim = Vec3(xDim / 2.0, 0, zDim / 2.0);
		Vec3 newMin = middle - halfDim, newMax = middle + halfDim;
		//create soccer field
		m_bbox = new AABBox(newMin, newMax);
		// goal dims
		float xGoalDim = xDim * 0.05, zGoalDim = zDim * 0.7;
		// Red goal
		m_TeamAGoal = new AABBox(newMax + Vec3(-xGoalDim * 0.25, 0, -(zDim + zGoalDim) / 2.0),
				newMax + Vec3(xGoalDim * 0.75, 0, -(zDim - zGoalDim) / 2.0));
		// Blue Goal
		m_TeamBGoal = new AABBox(newMin + Vec3(-xGoalDim * 0.75, 0, (zDim - zGoalDim) / 2.0),
				newMin + Vec3(xGoalDim * 0.25, 0, (zDim + zGoalDim) / 2.0));
		// update the ball's AABB if any
		if (m_Ball)
		{
			m_Ball->m_bbox = m_bbox;
		}
	}

	const AVGroup& allVehicles(void)
	{
		return (const AVGroup&) m_AllVehicles;
	}

///	unsigned int m_PlayerCountA;
///	unsigned int m_PlayerCountB;
	typename Player<Entity>::groupType TeamA;
	typename Player<Entity>::groupType TeamB;
	typename Player<Entity>::groupType m_AllPlayers;
	Ball<Entity> *m_Ball;

	typedef typename Player<Entity>::groupType::iterator iterator;

	AABBox *m_bbox;
	AABBox *m_TeamAGoal;
	AABBox *m_TeamBGoal;
	int junk;
	int m_redScore;
	int m_blueScore;
	//
	AVGroup m_AllVehicles;
};

//MicTestPlugIn pMicTestPlugIn;

// ----------------------------------------------------------------------------

} // ossup namespace

#endif /* PLUGIN_SOCCER_H_ */
