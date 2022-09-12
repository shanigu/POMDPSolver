package pomdp.environments;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Random;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.CreateBeliefSpaces;
import pomdp.algorithms.ValueIteration;
import pomdp.algorithms.pointbased.ForwardSearchValueIteration;
import pomdp.algorithms.pointbased.ForwardSearchValueIteration.HeuristicType;
import pomdp.environments.FactoredPOMDP.BeliefType;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.Logger;

public class SearchAndRescue extends FactoredPOMDP {

	protected int[][] m_aiMap; 
	protected int m_cX, m_cY;
	protected int m_iInitialX, m_iInitialY, m_cStartStates; 
	protected int m_cXBits, m_cYBits;
	
	private enum Action{
		North, East, South, West
	}

	/**
	 * Prints the rock sample map.
	 * the printed value is the height of the square.
	 *
	 */
	public void printMap(){
		int iX = 0, iY = 0;
		for( iY = 0 ; iY < m_cY ; iY++ ){
			for( iX = 0 ; iX < m_cX ; iX++ ){
				
				System.out.print( m_aiMap[iX][iY] );
			}
			System.out.println();
		}
	}
	
	public String getName(){
		return "SearchAndRescue_" + m_cX + "_" + m_cY;
	}
	
	public SearchAndRescue( int cX, int cY, int cRocks, BeliefType btFactored, boolean bUseSpecialADDForG, boolean bUseRelevantVariablesOnly ){
		this( cX, cY, getMap( cX, cY ), btFactored, bUseSpecialADDForG, bUseRelevantVariablesOnly );
	}
			
	public SearchAndRescue( int cX, int cY, BeliefType btFactored ){
		this( cX, cY, getMap( cX, cY ), btFactored, false, true );
	}
			
	private static int[][] getMap( int cX, int cY ) {
		int[][] aiMap = new int[cX][];
		if( cX == 8 && cY == 8 ){
			/*
			aiMap[0] = new int[]{ 0, 0, 0, 0, 0, 0, 0, 0 };
			aiMap[1] = new int[]{ 0, 0, 0, 1, 1, 0, 0, 0 };
			aiMap[2] = new int[]{ 0, 0, 1, 2, 2, 1, 0, 0 };
			aiMap[3] = new int[]{ 0, 0, 0, 1, 1, 0, 0, 0 };
			aiMap[4] = new int[]{ 0, 1, 0, 0, 0, 0, 1, 0 };
			aiMap[5] = new int[]{ 1, 2, 1, 0, 0, 1, 2, 1 };
			aiMap[6] = new int[]{ 0, 1, 0, 0, 0, 0, 1, 0 };
			aiMap[7] = new int[]{ 0, 0, 0, 0, 0, 0, 0, 0 };
			*/
			aiMap[0] = new int[]{ 0, 0, 0, 0, 0, 0, 0, 0 };
			aiMap[1] = new int[]{ 0, 1, 0, 2, 0, 1, 0, 0 };
			aiMap[2] = new int[]{ 0, 0, 0, 1, 0, 0, 0, 0 };
			aiMap[3] = new int[]{ 0, 0, 1, 2, 1, 0, 0, 0 };
			aiMap[4] = new int[]{ 0, 0, 0, 1, 0, 0, 0, 0 };
			aiMap[5] = new int[]{ 0, 1, 0, 0, 0, 1, 0, 0 };
			aiMap[6] = new int[]{ 0, 0, 0, 0, 0, 0, 0, 0 };
			aiMap[7] = new int[]{ 0, 0, 0, 0, 0, 0, 0, 0 };
			
		}
		return aiMap;
	}

	public SearchAndRescue( int cX, int cY ){
		this( cX, cY, getMap( cX, cY ), BeliefType.Factored, false, true );
	}
			
	public SearchAndRescue( int cX, int cY, int[][] aiMap, BeliefType btFactored, boolean bUseSpecialADDForG, boolean bUseRelevantVariablesOnly ){
		super( ( log( cX ) + log( cY ) ) * 2 + 1, 4, 9, btFactored, bUseSpecialADDForG, bUseRelevantVariablesOnly );
		m_dGamma = 0.9;
		m_cXBits = log( cX ); 
		m_cYBits = log( cY );
		m_cX = cX;
		m_cY = cY;
		m_iInitialX = 0; //change here to get a different starting place
		m_iInitialY = m_cY / 2; //change here to get a different starting place
		m_cActions = 4; //north east south west 
		m_cObservations = 9; //human location
		m_dGamma = 0.95;
		m_aiMap = aiMap;

		m_cStartStates = 0;
		for( int[] a : m_aiMap )
			for( int i : a )
				if( i == 0 )
					m_cStartStates++;
		
		initADDs();
		
		printMap();
		double dMdpAdr = computeMDPAverageDiscountedReward( 200, 250 );
		System.out.println( "MDP ADR = " + dMdpAdr );
	}

	private Action intToAction( int iAction ){
		if( iAction == 0 )
			return Action.North;
		if( iAction == 1 )
			return Action.East;
		if( iAction == 2 )
			return Action.South;
		if( iAction == 3 )
			return Action.West;
		return null;
	}
	
	protected int stateToIndex( int iAgentX, int iAgentY, int iHumanX, int iHumanY, boolean bDone ){
		int iState = iAgentY;
		iState = ( iState << m_cYBits ) + iAgentX;
		iState = ( iState << m_cXBits ) + iHumanY;
		iState = ( iState << m_cYBits ) + iHumanX;
		iState = ( iState << 1 );
		if( bDone )
			iState++;
		return iState;
	}
	
	protected int stateToIndex( boolean[] abState ){
		int iState = 0;
		int iVariable = abState.length - 1, iBit = 0;
		for( iBit = 0 ; iBit < ( m_cXBits + m_cYBits ) * 2 ; iBit++ ){
			iState *= 2;
			if( abState[iVariable] )
				iState += 1;
			iVariable--;
		}	
		return iState;
	}

	
	protected int stateToIndex( int[] aiStateVariableIndexes, boolean[] abStateVariableValues ){
		int cVariables = getStateVariablesCount();
		int iState = 0;
		int iVariable = cVariables - 1, idx = aiStateVariableIndexes.length - 1;
		for( iVariable = cVariables - 1 ; iVariable >= 0 ; iVariable-- ){
			iState *= 2;
			if( ( idx >= 0 ) && ( aiStateVariableIndexes[idx] == iVariable ) ){
				if( abStateVariableValues[idx] )
					iState += 1;
				idx--;
			}
		}		
		return iState;
	}

	public String indexToString( int iState ){
		boolean bDone = isTerminalState( iState );
		int iAgentX = getAgentX( iState ), iAgentY = getAgentY( iState );
		int iHumanX = getHumanX( iState ), iHumanY = getHumanY( iState );
		String sState = "<A:" + iAgentX + "," + iAgentY + ",H:" + iHumanX + "," + iHumanY + ">";
		return sState;
	}
	
	//human coordinates are always relevant
	public int[] getRelevantVariables( int iAction ){
		/*
		int[] aiRelevant = null;
		int iStateVariable = 0;
		Action a = intToAction( iAction );
		if( a == Action.East || a == Action.West ){
			aiRelevant = new int[m_cXBits * 2 + m_cYBits];
			for( iStateVariable = 0 ; iStateVariable < m_cXBits ; iStateVariable++ )
				aiRelevant[iStateVariable + m_cXBits + m_cYBits] = m_cXBits + m_cYBits + iStateVariable;
		}
		else if( a == Action.North || a == Action.South ){
			aiRelevant = new int[m_cYBits * 2 + m_cXBits];
			for( iStateVariable = 0 ; iStateVariable < m_cYBits ; iStateVariable++ )
				aiRelevant[iStateVariable + m_cXBits + m_cYBits] = m_cXBits * 2 + m_cYBits + iStateVariable;
		}
		for( iStateVariable = 0 ; iStateVariable < m_cXBits + m_cYBits ; iStateVariable++ )
			aiRelevant[iStateVariable] = iStateVariable;
		return aiRelevant;
		*/
		return getAllVariables();
	}

	protected int[] getAllVariables(){
		int[] aiVars = new int[getStateVariablesCount()];
		int iVar = 0;
		for( iVar = 0 ; iVar < getStateVariablesCount() ; iVar++ )
			aiVars[iVar] = iVar;
		return aiVars;
	}
	
	protected int[] getObservationRelevantVariables( int iAction ){
		return getAllVariables();
	}

	protected int[] getRewardRelevantVariables( int iAction ){
		return getAllVariables();
	}

	public boolean[] indexToState( int iState ){
		boolean[] abState = new boolean[getStateVariablesCount()];
		int iVariable = 0, iBit = 0;
		for( iBit = 0 ; iBit < abState.length ; iBit++ ){
			abState[iVariable] = ( iState % 2 == 1 );
			iVariable++;
			iState /= 2;
		}
		return abState;
	}
	
	private int getAgentX( int iState ){
		int iBit = 0, iX = 0, iPower = 1;
		for( iBit = 0 ; iBit < m_cXBits + m_cYBits + 1 ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			iX += ( iState % 2 ) * iPower;
			iPower *= 2;
			iState /= 2;
		}
		return iX;
	}
	
	private int getAgentY( int iState ){
		int iBit = 0, iY = 0, iPower = 1;
		for( iBit = 0 ; iBit < m_cXBits * 2 + m_cYBits + 1 ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			iY += ( iState % 2 ) * iPower;
			iPower *= 2;
			iState /= 2;
		}
		return iY;
	}

	private int getHumanX( int iState ){
		int iBit = 0, iX = 0, iPower = 1;
		iState /= 2;
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			iX += ( iState % 2 ) * iPower;
			iPower *= 2;
			iState /= 2;
		}
		return iX;
	}
	
	private int getHumanY( int iState ){
		int iBit = 0, iY = 0, iPower = 1;
		for( iBit = 0 ; iBit < m_cXBits + 1 ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			iY += ( iState % 2 ) * iPower;
			iPower *= 2;
			iState /= 2;
		}
		return iY;
	}
/*
	private int getX( int[] aiVariableIds, boolean[] abVariableValues ){
		int i = 0, iX = 0;
		for( i = aiVariableIds.length - 1 ; i >= 0 ; i-- ){		
			if( aiVariableIds[i] < m_cXBits ){
				iX *= 2;
				if( abVariableValues[i] )
					iX += 1;
				else
					iX += 0;
			}
		}
		return iX;
	}
	
	private int getY( int[] aiVariableIds, boolean[] abVariableValues ){
		int i = 0, iY = 0;
		for( i = aiVariableIds.length - 1 ; i >= 0 ; i-- ){		
			if( ( aiVariableIds[i] >= m_cXBits ) && ( aiVariableIds[i] < m_cXBits + m_cYBits ) ){
				iY *= 2;
				if( abVariableValues[i] )
					iY += 1;
				else
					iY += 0;
			}
		}
		return iY;
	}
/*
	private int getX( int[] aiVariableIds, boolean[] abVariableValues ){
		int i = 0, iX = 0;
		for( i = aiVariableIds.length - 1 ; i >= 0 ; i-- ){		
			if( aiVariableIds[i] < m_cXBits ){
				iX *= 2;
				if( abVariableValues[i] )
					iX += 1;
				else
					iX += 0;
			}
		}
		return iX;
	}
	
	private int getY( int[] aiVariableIds, boolean[] abVariableValues ){
		int i = 0, iY = 0;
		for( i = aiVariableIds.length - 1 ; i >= 0 ; i-- ){		
			if( ( aiVariableIds[i] >= m_cXBits ) && ( aiVariableIds[i] < m_cXBits + m_cYBits ) ){
				iY *= 2;
				if( abVariableValues[i] )
					iY += 1;
				else
					iY += 0;
			}
		}
		return iY;
	}
*/
	
	private double moveTr( int iXStart, int iXEnd, int iYStart, int iYEnd, Action a ){
		switch( a ){
		case North: //move north
			if( iXStart != iXEnd )
				return 0.0;
			if( ( iYStart + 1 == iYEnd ) || ( ( iYStart == m_cY - 1 ) && ( iYEnd == iYStart ) ) )
				return 1.0;
			return 0.0;
		case East: //move east
			if( iYStart != iYEnd )
				return 0.0;
			if( ( iXStart + 1 == iXEnd ) || ( ( iXStart == m_cX - 1 ) && ( iXEnd == iXStart ) ) )
				return 1.0;
			return 0.0;
		case South: //move south
			if( iXStart != iXEnd )
				return 0.0;
			if( ( iYStart - 1 == iYEnd ) || ( ( iYStart == 0 ) && ( iYEnd == iYStart ) ) )
				return 1.0;
			return 0.0;
		case West: //move west
			if( iYStart != iYEnd )
				return 0.0;
			if( ( iXStart - 1 == iXEnd ) || ( ( iXStart == 0 ) && ( iXEnd == iXStart ) ) )
				return 1.0;
			return 0.0;
		}
		return -1;
	}
	
	public double tr( int iStartState, int iAction, int iEndState ){
		int iAgentXStart = getAgentX( iStartState ), iAgentYStart = getAgentY( iStartState );
		int iAgentXEnd = getAgentX( iEndState ), iAgentYEnd = getAgentY( iEndState );
		int iHumanXStart = getHumanX( iStartState ), iHumanYStart = getHumanY( iStartState );
		int iHumanXEnd = getHumanX( iEndState ), iHumantYEnd = getHumanY( iEndState );
		boolean bStartDone = isTerminalState( iStartState );
		boolean bEndDone = iAgentXEnd == iHumanXEnd && iAgentYEnd == iHumantYEnd;
		Action a = intToAction( iAction );
		
		//an implementation for stationary human
		if( ( iHumanXStart != iHumanXEnd ) || ( iHumanYStart != iHumantYEnd ) )
			return 0.0;
		
		if( bStartDone && !isTerminalState( iEndState ) )
			return 0.0;
		
		if( !bStartDone && isTerminalState( iEndState ) && !bEndDone )
			return 0.0;
		
		return moveTr( iAgentXStart, iAgentXEnd, iAgentYStart, iAgentYEnd, a ); 			
	}
	

	//probability that state variable will have the value TRUE if action a is taken at state s
	protected double stateVariableTransition( int iState, int iAction, int iStateVariable ){
		/*
		Action a = intToAction( iAction );
		int iAgentX = getAgentX( iState );
		int iAgentY = getAgentY( iState );
		int iHumanX = getHumanX( iState );
		int iHumanY = getHumanY( iState );
		if( iStateVariable < m_cXBits ){ //human x coord state variable
			if( ( iHumanX >> iStateVariable ) % 2 == 1 ) //implementation for stationary human
				return 1.0;
			else
				return 0.0;
		}
		else if( iStateVariable < m_cXBits + m_cYBits ){//human y coord state variable
			if( ( iHumanY >> iStateVariable ) % 2 == 1 ) //implementation for stationary human
				return 1.0;
			else
				return 0.0;
		}
		else if( iStateVariable < m_cXBits * 2 + m_cYBits ){// agent x coord state variable
			if( a == Action.East || a == Action.West ){ //east and west
				iAgentX = applyXMove( iAgentX, a ); //change the x to reflect an execution of a
			}// otherwise x stays the same
			if( ( iAgentX >> iStateVariable ) % 2 == 1 )
				return 1.0;
			else
				return 0.0;
	
		}
		else if( iStateVariable < m_cXBits * 2 + m_cYBits * 2 ){// agent y coord state variable
			if( a == Action.North || a == Action.South ){ //north and south
				iAgentY = applyYMove( iAgentY, a );//change the y to reflect an execution of a
			}// otherwise y stays the same
			if( ( iAgentY >> ( iStateVariable - m_cXBits ) ) % 2 == 1 )
				return 1.0;
			else
				return 0.0;			
		}
		*/
		return -1.0;
	}

	private int applyXMove( int iX, Action a ){
		switch( a ){
		case East: //move east
			if( iX == m_cX - 1 )
				return m_cX - 1; 
			return iX + 1;
		case West: //move west
			if( iX == 0 )
				return 0;
			return iX - 1;
		}
		return iX;
	}

	private int applyYMove( int iY, Action a ){
		switch( a ){
		case North: //move north
			if( iY == m_cY - 1 )
				return m_cY - 1;
			return iY + 1;
		case South: //move south
			if( iY == 0 )
				return 0;
			return iY - 1;
		}
		return iY;
	}

	public double R( int iStartState, int iAction ){
		if( isTerminalState( iStartState ) )
			return 0.0;
		int iAgentX = getAgentX( iStartState );
		int iAgentY = getAgentY( iStartState );
		int iHumanX = getHumanX( iStartState );
		int iHumanY = getHumanY( iStartState );
		if( iAgentX == iHumanX && iAgentY == iHumanY )
			return 10.0;
		
		Action a = intToAction( iAction );
		int iAgentEndX = applyXMove( iAgentX, a );
		int iAgentEndY = applyYMove( iAgentY, a );
		
		int iDelta = m_aiMap[iAgentX][iAgentY] - m_aiMap[iAgentEndX][iAgentEndY];
		if( iDelta > 0 )
			iDelta = 0;
		
		return iDelta / 2.0;
	}
	
	public double R( int iStartState, int iAction, int iEndState ){
		return R( iStartState, iAction ); 
	}
	
	public double R( int iStartState ){
		return -1;
	}
		
	protected int getNoisyObservation( int iObservation, double dProbability ){
		if( m_rndGenerator.nextDouble() < dProbability )
			return iObservation;
		else{
			int iRandomObservation = m_rndGenerator.nextInt( m_cObservations - 1 );
			if( iRandomObservation >= iObservation )
				iRandomObservation++;
			return iRandomObservation;
		}
	}
	
	private double getObservationSuccessProbability( int iState ){
		int iX = getAgentX( iState ), iY = getAgentY( iState );
		int iHeight = m_aiMap[iX][iY];
		if( iHeight == 0 )
			return 1.0 / m_cObservations;
		if( iHeight == 1 )
			return 0.75;
		if( iHeight == 2 )
			return 1.0;
		return 0.0;
	}
	
	public int observe( int iAction, int iEndState ){
		double dPr = getObservationSuccessProbability( iEndState );
		int iDirection = computeAgentHumanDirection( iEndState ); 
		return getNoisyObservation( iDirection, dPr );
	}
	
	private int sign( int x )
	{
		if( x < 0 )
			return -1;
		if( x > 0 )
			return 1;
		return 0;
	}
	
	private int computeAgentHumanDirection( int iState ) {
		int iAgentX = getAgentX( iState );
		int iAgentY = getAgentY( iState );
		int iHumanX = getHumanX( iState );
		int iHumanY = getHumanY( iState );
		int iXDelta = sign( iHumanX - iAgentX ) + 1;
		int iYDelta = sign( iHumanY - iAgentY ) + 1;
		int iDirection = iYDelta * 3 + iXDelta;
		if( iDirection < 0 || iDirection >= m_cObservations )
			System.out.println();
		return iDirection;
	}

	public double O( int iAction, int iEndState, int iObservation ){
		int iDirection = computeAgentHumanDirection( iEndState );
		double dPr = getObservationSuccessProbability( iEndState );
		if( iDirection == iObservation )
			return dPr;
		return ( 1.0 - dPr ) / ( m_cObservations - 1 );
	}
	
	public Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iStartState, int iAction ) {
		Map<Integer,Double> mTransitions = null;//m_amTransitions[iStartState][iAction];
		int iEndState = 0;
		double dTr = 0.0;
		if( true || mTransitions == null ){
			mTransitions = new TreeMap<Integer,Double>();
			iEndState = execute( iAction, iStartState );
			dTr = tr( iStartState, iAction, iEndState );
			mTransitions.put( iEndState, dTr );
			//m_amTransitions[iStartState][iAction] = mTransitions;
		}
		return mTransitions.entrySet().iterator();
	}

	public Collection<Entry<Integer,Double>> getNonZeroBackwardTransitions( int iAction, int iEndState ) {
		Map<Integer,Double> mTransitions = null;//m_amBackwardTransitions[iAction][iEndState];
		int[] iStartStates = null;
		double dTr = 0.0;
		if( true || mTransitions == null ){
			mTransitions = new TreeMap<Integer,Double>();
			iStartStates = unExecute( iAction, iEndState );
			for( int iStartState : iStartStates ){
				dTr = tr( iStartState, iAction, iEndState );
				mTransitions.put( iEndState, dTr );
			}
		}
		return mTransitions.entrySet();
	}

	//begin at leftmost cloumn in the middle with at least one good rock
	public int chooseStartState(){
		int iAgentX = m_iInitialX, iAgentY = m_iInitialY;
		int iHumanX = 0;
		int iHumanY = 0;
		int iStartStateIdx = m_rndGenerator.nextInt( m_cStartStates );
		for( iHumanX = 0 ; iHumanX < m_cX && iStartStateIdx >= 0 ; iHumanX++ ){
			for( iHumanY = 0 ; iHumanY < m_cY && iStartStateIdx >= 0 ; iHumanY++ ){
				if( m_aiMap[iHumanX][iHumanY] == 0 )
					iStartStateIdx--;
			}
		}
		
		if( m_aiMap[iHumanX - 1][iHumanY - 1] != 0 )
			System.out.println( "BUGBUG" );
		
		return stateToIndex( iAgentX, iAgentY, iHumanX - 1, iHumanY - 1, false );
	}

	public double probStartState( int iState ){
		if( isTerminalState( iState ) )
			return 0.0;
		int iAgentX = getAgentX( iState );
		int iAgentY = getAgentY( iState );
		int iHumanX = getHumanX( iState );
		int iHumanY = getHumanY( iState );
		if( m_aiMap[iHumanX][iHumanY] > 0 )
			return 0.0;
		if( ( iAgentX == m_iInitialX ) && ( iAgentY == m_iInitialY ) )
			return 1.0 / m_cStartStates;
		return 0.0;
	}
	
	public int execute( int iAction, int iState ){
		boolean bDone = isTerminalState( iState );
		int iAgentX = getAgentX( iState ), iAgentY = getAgentY( iState );
		int iHumanX = getHumanX( iState ), iHumanY = getHumanY( iState );
		Action a = intToAction( iAction );
		if( ( iAgentX == iHumanX ) && ( iAgentY == iHumanY ) )
			bDone = true;
		switch( a ){
		case North:
			if( iAgentY < m_cY - 1 )
				iAgentY++;
			break;
		case South:
			if( iAgentY > 0 )
				iAgentY--;
			break;
		case East:
			if( iAgentX < m_cX - 1 )
				iAgentX++;
			break;
		case West:
			if( iAgentX > 0 )
				iAgentX--;
			break;
		}
		return stateToIndex( iAgentX, iAgentY, iHumanX, iHumanY, bDone );
	}
	
	public int[] unExecute( int iAction, int iEndState ){
		//BUGBUG - currently does not work. There is a problem with move actions
		return null;
	}
	
	public boolean isTerminalState( int iState ){
		return ( iState % 2 ) == 1;
	}
	
	public boolean terminalStatesDefined(){
		return true;
	}
	
	public double getMaxMinR(){
		return 0.0;
	}

	public int getStartStateCount() {
		return m_cX * m_cY;
	}
	
	public String getStateName( int iState ){
		int iAgentX = getAgentX( iState ), iAgentY = getAgentY( iState );
		int iHumanX = getHumanX( iState ), iHumanY = getHumanY( iState );
		String sState = "<A:" + iAgentX + "," + iAgentY + ",H:" + iHumanX + "," + iHumanY + ">";
		return sState;
	}
	
	public String getActionName( int iAction ){
		String sActionName = intToAction( iAction ).toString();
		return sActionName;
	}
	
	protected double stateVariableTransition( int[] aiStateVariableIndexes, boolean[] abStateVariableValuesBefore, int iAction, boolean[] abStateVariableValuesAfter ){
		int iStartState = stateToIndex( aiStateVariableIndexes, abStateVariableValuesBefore );
		int iEndState = stateToIndex( aiStateVariableIndexes, abStateVariableValuesAfter );
		return tr( iStartState, iAction, iEndState );
	}	
	protected double stateVariableObservation( int[] aiStateVariableIndexes, int iAction, boolean[] abStateVariableValues, int iObservation ){
		int iEndState = stateToIndex( aiStateVariableIndexes, abStateVariableValues );
		return O( iAction, iEndState, iObservation );
	}
	protected String getObservationName() {
		return null;
	}

	public String getObservationName( int iObservation ) {
		int iXDirection = iObservation / 3 - 1;
		int iYObservation = iObservation % 3 - 1;
		String sDirection = "";
		if( iYObservation == -1 )
			sDirection = "S";
		if( iYObservation == 1 )
			sDirection = "N";
		if( iXDirection == -1 )
			sDirection += "E";
		if( iXDirection == 1 )
			sDirection += "W";
		if( sDirection.length() == 0 )
			return "?";
		return sDirection;
	}

	protected String getVariableName( int iVariable ) {
		if( iVariable < m_cXBits )
			return "HumanX" + iVariable;
		else if( iVariable < m_cXBits + m_cYBits ){
			return "HumanY" + ( iVariable - m_cXBits );
		}
		else if( iVariable < m_cXBits * 2 + m_cYBits ){
			return "AgentX" + ( iVariable - m_cXBits - m_cYBits );
		}
		else if( iVariable < m_cXBits * 2 + m_cYBits * 2 ){
			return "AgentY" + ( iVariable - m_cXBits * 2 - m_cYBits );
		}
		return null;
	}

	public double getInitialVariableValueProbability( int iVariable, boolean bValue ){
		if( iVariable < m_cXBits ){
			if( getBit( m_iInitialX, iVariable ) == bValue )
				return 1.0;
			return 0.0;
		}
		else if( iVariable < m_cXBits + m_cYBits ){
			if( getBit( m_iInitialY, iVariable - m_cXBits ) == bValue )
				return 1.0;
			return 0.0;
		}
		return 0.5;
	}

	protected boolean relevantTransitionVariable( int iAction, int iVariable ){
		/*
		Action a = intToAction( iAction );
		if( a == Action.East || a == Action.West ){
			if( ( iVariable >= m_cXBits ) && ( iVariable < m_cXBits + m_cYBits ) ) // only X bits are relevant when moving over X axis
				return true;
		}
		else if( a == Action.North || a == Action.South ){
			if( ( iVariable >= m_cXBits ) && ( iVariable < m_cXBits + m_cYBits ) )
				return true;// only Y bits are relevant when moving over Y axis
		}
		*/
		return false;
	}

	
	private boolean getBit( int iNumber, int iDigit ){
		iNumber = iNumber >> iDigit;
		return iNumber % 2 == 1;
	}
	
	
	public double transitionGivenRelevantVariables( int iAction, int iVariable, boolean bValue, int[] aiRelevantVariables, boolean[] abValues ) {
/*
		int iX = 0, iY = 0;
		int iRock = 0;
		Action a = intToAction( iAction );

		if( iAction < 4 ){ //move action
			if( a == Action.North ){
				iY = getY( aiRelevantVariables, abValues );
				if( iY < m_cY - 1 )
					iY++;
				if( getBit( iY, iVariable - m_cXBits ) == bValue )
					return 1.0;
				return 0.0;
			}
			if( a == Action.South ){
				iY = getY( aiRelevantVariables, abValues );
				if( iY > 0 )
					iY--;
				if( getBit( iY, iVariable - m_cXBits ) == bValue )
					return 1.0;
				return 0.0;
			}
			if( a == Action.East ){
				iX = getX( aiRelevantVariables, abValues );
				if( iX < m_cX - 1 )
					iX++;
				if( getBit( iX, iVariable ) == bValue )
					return 1.0;
				return 0.0;
			}
			if( a == Action.West ){
				iX = getX( aiRelevantVariables, abValues );
				if( iX > 0 )
					iX--;
				if( getBit( iX, iVariable ) == bValue )
					return 1.0;
				return 0.0;
			}
		}
		
*/
		return 0.0;
	}	
	
	public double observationGivenRelevantVariables( int iAction, int iObservation, int[] aiRelevantVariables, boolean[] abValues ) {
/*
		int iX = 0, iY = 0;
		int iRock = 0;
		Action a = intToAction( iAction );
		
		if( iAction < 4 ){ //move action
			if( iObservation == 0 )
				return 1.0;
			else
				return 0.0;
		}
		
		if( a == Action.Check ){
			if( a == Action.Check ){
				iRock = iAction - 4;
				iX = getX( aiRelevantVariables, abValues );
				iY = getY( aiRelevantVariables, abValues );
				int iRockX = m_aiMap[iRock][0];
				int iRockY = m_aiMap[iRock][1];
				double dPr = getCheckSuccessProbability( iX, iY, iRockX, iRockY );
						
				for( int i = 0 ; i < aiRelevantVariables.length ; i++ ){
					if( aiRelevantVariables[i] == m_cXBits + m_cYBits + iRock ){
						if( abValues[i] == true ){
							if( iObservation == 1 )
								return dPr;
							else
								return 1 - dPr;
						}
						else{
							if( iObservation == 0 )
								return dPr;
							else
								return 1 - dPr;
						}			
					}
				}
			}
		}
		
		if( a == Action.Sample ){
			if( iObservation == 0 )
				return 1.0;
			else
				return 0.0;
		}
*/
		return 0.0;
	}	
	
	public double rewardGivenRelevantVariables( int iAction, int[] aiRelevantVariables, boolean[] abValues ) {
		/*
		int iX = 0, iY = 0;
		int iRock = 0;
		Action a = intToAction( iAction );
		int iRockX = 0, iRockY = 0;
		
		if( iAction < 4 || a == Action.Check ){ //move action
			return 0.0;
		}
		
		if( a == Action.Sample ){
			iX = getX( aiRelevantVariables, abValues );
			iY = getY( aiRelevantVariables, abValues );
			
			for( iRock = 0 ; iRock < m_cRocks ; iRock++ ){
				iRockX = m_aiMap[iRock][0];
				iRockY = m_aiMap[iRock][1];
				if( iX == iRockX && iY == iRockY ){
					for( int i = 0 ; i < aiRelevantVariables.length ; i++ ){
						if( aiRelevantVariables[i] == m_cXBits + m_cYBits + iRock ){
							if( abValues[i] == true ){
								return 10.0;
							}			
							else{
								return -10.0;
							}
						}
					}					
				}
			}
			
			return -10.0;//sample where no rock exists
		}
		*/
		return 0.0;
	}	
	
	private boolean hasSameValue( int iVariable, boolean bValue, int[] aiRelevantVariables, boolean[] abValues ) {
		for( int i = 0 ; i < aiRelevantVariables.length ; i++ ){
			if( aiRelevantVariables[i] == iVariable ){
				if( bValue == abValues[i] )
					return true;
				return false;
			}
		}
		return true;
	}

	
	protected double getInitialVariableValueProbability( int iVariable, int iValue ) {
		/*
		if( ( iVariable == 0 ) || ( iVariable == 1 ) ){
			if( iValue == 0 )
				return 1.0;
			else
				return 0.0;
		}
		if( ( iValue == 0 ) || ( iValue == 1 ) )
			return 0.5;
			*/
		return 0.0;
	}

	
	protected int[] getObservationRelevantVariablesMultiValue( int iAction ) {
		/*
		if( ( iAction < 4 ) || ( iAction == m_cActions - 1 ) )
			return new int[0];
		return new int[]{ 0, 1, iAction - 4 + 2 };
		*/
		return null;
	}

	
	protected int getRealStateVariableCount() {
		return -1;
	}

	
	protected String getRealVariableName(int iVariable) {
		return null;
	}

	
	protected int[] getRelevantVariablesMultiValue( int iAction, int iVariable ){
		return new int[]{ iVariable };
	}

	
	protected int getValueCount( int iVariable ) {
		if( iVariable % 2 == 0 )
			return m_cX;
		if( iVariable % 2 == 1 )
			return m_cY;
		return 0;
	}

	
	protected String getValueName( int iVariable, int iValue ){
		/*
		if( iVariable == 0 )
			return "X" + iValue;
		if( iVariable == 1 )
			return "Y" + iValue;
		if( iValue == 0)
			return "bad";
		else
			return "good";
			*/
		return null;
	}

	
	protected double observationGivenRelevantVariablesMultiValue( int iAction, int iObservation, int[] aiRelevantVariables, int[] aiValues ) {
		/*
		if( ( iAction < 4 ) || ( iAction == m_cActions - 1 ) ){
			if( iObservation == 0 )
				return 0.0;
			else
				return 1.0;
		}
		int iRock = iAction - 4;
		int iX = aiValues[0], iY = aiValues[1];
		int iRockX = m_aiMap[iRock][0];
		int iRockY = m_aiMap[iRock][1];
		double dO = getCheckSuccessProbability( iX, iY, iRockX, iRockY );
		if( iObservation == aiValues[2] )
			return dO;
		else
			return 1 - dO;
			*/
		return 0.0;
	}

	
	protected boolean relevantTransitionRealVariable( int iAction, int iVariable ) {
		/*
		Action a = intToAction( iAction );
		if( a.equals( Action.West ) || a.equals( Action.East ) ){
			return ( iVariable == 0 );
		}
		if( a.equals( Action.North ) || a.equals( Action.South ) ){
			return ( iVariable == 1 );
		}
		if( a.equals( Action.Sample ) ){
			return true;
		}
		else{
			return false;
		}
		*/
		return false;
	}


	
	protected double rewardGivenRelevantVariablesMultiValue( int iAction, int[] aiRelevantVariables, int[] aiValues ) {
		/*
		if( iAction < m_cActions - 1 )
			return 0.0;
		int iX = aiValues[0], iY = aiValues[1];
		int iRock = getRockAt( iX, iY );
		if( iRock > -1 ){
			int iRockX = m_aiMap[iRock][0];
			int iRockY = m_aiMap[iRock][1];
			if( iRockX == iX && iRockY == iY ){
				if( aiValues[iRock + 2] == 1 )
					return 10.0;
			}
		}
		*/
		return -10.0;
	}

	
	protected double transitionGivenRelevantVariablesMultiValue( int iAction, int iVariable, int iValue, int[] aiRelevantVariables, int[] aiValues ){
		/*
		Action a = intToAction( iAction );
		if( a.equals( Action.Check ) ){
			if( iValue == getVariableValue( aiRelevantVariables, aiValues, iVariable ) ){
				return 1.0;
			}
			return 0.0;
		}
		if( a.equals( Action.East ) || a.equals( Action.West ) ){
			if( iVariable != 0 ){
				if( iValue == getVariableValue( aiRelevantVariables, aiValues, iVariable ) ){
					return 1.0;
				}
				return 0.0;				
			}
			int iNewValue = getVariableValue( aiRelevantVariables, aiValues, iVariable );
			if( a.equals( Action.East ) )
				iNewValue++;
			if( a.equals( Action.West ) )
				iNewValue--;
			if( iNewValue == m_cX )
				iNewValue--;
			if( iNewValue < 0 )
				iNewValue++;
			
			if( iNewValue == iValue )
				return 1.0;
			return 0.0;
		}
		if( a.equals( Action.North ) || a.equals( Action.South ) ){
			if( iVariable != 1 ){
				if( iValue == getVariableValue( aiRelevantVariables, aiValues, iVariable ) ){
					return 1.0;
				}
				return 0.0;				
			}
			int iNewValue = getVariableValue( aiRelevantVariables, aiValues, iVariable );
			if( a.equals( Action.North ) )
				iNewValue++;
			if( a.equals( Action.South ) )
				iNewValue--;
			if( iNewValue == m_cX )
				iNewValue--;
			if( iNewValue < 0 )
				iNewValue++;
			
			if( iNewValue == iValue )
				return 1.0;
			return 0.0;
		}
		if( a.equals( Action.Sample ) ){
			int iX = getVariableValue( aiRelevantVariables, aiValues, 0 );
			int iY = getVariableValue( aiRelevantVariables, aiValues, 1 );
			int iRock = getRockAt( iX, iY );
			if( iRock < 0 ){
				if( iValue == getVariableValue( aiRelevantVariables, aiValues, iVariable ) ){
					return 1.0;
				}
				return 0.0;								
			}
			if( iVariable != iRock + 2 ){
				if( iValue == getVariableValue( aiRelevantVariables, aiValues, iVariable ) ){
					return 1.0;
				}
				return 0.0;								
			}
			if( iValue == 0 ){
				return 1.0;
			}
			return 0.0;
		}
		*/
		return Double.POSITIVE_INFINITY;
	}

	
	protected int[] getRewardRelevantVariablesMultiValue( int iAction ){
		/*
		if( iAction < m_cActions - 1 )
			return new int[0];
		int[] aiVars = new int[m_cRocks + 2];
		for( int iVar = 0 ; iVar < aiVars.length ; iVar++ ){
			aiVars[iVar] = iVar;
		}
		return aiVars;
		*/
		return null;
	}

	
	public int[] getIndependentComponentVariables( int iComponent ) {
		/*
		int[] aiVariables = null;
		int i = 0;
		if( iComponent == 0 ){
			aiVariables = new int[m_cXBits];
			for( i = 0 ; i < m_cXBits ; i++ ){
				aiVariables[i] = i;
			}
		}
		else if( iComponent == 1 ){
			aiVariables = new int[m_cYBits];
			for( i = 0 ; i < m_cYBits ; i++ ){
				aiVariables[i] = m_cXBits + i;
			}
		}
		else{
			aiVariables = new int[]{ m_cXBits + m_cYBits + iComponent - 2 };
		}
		return aiVariables;
		*/
		return null;
	}

	
	public int getIndependentComponentsCount() {
		return 4;
	}

	
	public double getInitialComponenetValueProbability( int iComponent, int iValue ){
		/*
		if( iComponent == 0 ){
			if( iValue == m_iInitialX )
				return 1.0;
			return 0.0;
		}
		else if( iComponent == 1 ){
			if( iValue == m_iInitialY )
				return 1.0;
			return 0.0;
		}
		else{
			return 0.5;
		}
		*/
		return -1.0;
	}
	
	
	public double transitionGivenRelevantVariables( int iAction, int[] aiComponent, boolean[] abComponentValues, 
													int[] aiRelevantVariables, boolean[] abRelevantValues ){
		/*
		int iX = 0, iY = 0, iVariable = 0;
		int iRock = 0;
		Action a = intToAction( iAction );
		
		if( iAction < 4 ){ //move action
			if( ( a == Action.North ) || ( a == Action.South ) ){
				iY = getY( aiRelevantVariables, abRelevantValues );
				if( a == Action.North ){
					if( iY < m_cY - 1 )
						iY++;
				}
				if( a == Action.South ){
					if( iY > 0 )
						iY--;
				}
				for( iVariable = 0 ; iVariable < aiComponent.length ; iVariable++ ){
					if( ( aiComponent[iVariable] >= m_cXBits ) && ( aiComponent[iVariable] < m_cXBits + m_cYBits ) ){
						if( getBit( iY, iVariable ) != abComponentValues[iVariable] )
							return 0.0;
					}
					else{
						if( !hasSameValue( aiComponent[iVariable], abComponentValues[iVariable], 
								aiRelevantVariables, abRelevantValues ) )
							return 0.0;
					}
				}
				return 1.0;
			}
			if( ( a == Action.West ) || ( a == Action.East ) ){
				iX = getX( aiRelevantVariables, abRelevantValues );
				if( a == Action.East ){
					if( iX < m_cX - 1 )
						iX++;
				}
				if( a == Action.West ){
					if( iX > 0 )
						iX--;
				}
				for( iVariable = 0 ; iVariable < aiComponent.length ; iVariable++ ){
					if( aiComponent[iVariable] < m_cXBits ){
						if( getBit( iX, iVariable ) != abComponentValues[iVariable] )
							return 0.0;
					}
					else{
						if( !hasSameValue( aiComponent[iVariable], abComponentValues[iVariable], 
								aiRelevantVariables, abRelevantValues ) )
							return 0.0;
					}
				}
				return 1.0;
			}
		}
		
		if( a == Action.Check ){
			for( iVariable = 0 ; iVariable < aiComponent.length ; iVariable++ ){
				if( !hasSameValue( aiComponent[iVariable], abComponentValues[iVariable], 
						aiRelevantVariables, abRelevantValues ) )
					return 0.0;
			}
			return 1.0;
		}
		
		if( a == Action.Sample ){
			for( iVariable = 0 ; iVariable < aiComponent.length ; iVariable++ ){
				if( aiComponent[iVariable] < m_cXBits + m_cYBits ){
					if( !hasSameValue( aiComponent[iVariable], abComponentValues[iVariable], 
							aiRelevantVariables, abRelevantValues ) )
						return 0.0;
				}
				else{
					iRock = aiComponent[iVariable] - m_cXBits - m_cYBits;
					iX = getX( aiRelevantVariables, abRelevantValues );
					iY = getY( aiRelevantVariables, abRelevantValues );
					if( ( m_aiMap[iRock][0] == iX ) && ( m_aiMap[iRock][1] == iY ) ){
						if( abComponentValues[iVariable] == true )
							return 0.0;
					}
					else{
						if( !hasSameValue( aiComponent[iVariable], abComponentValues[iVariable], 
								aiRelevantVariables, abRelevantValues ) )
							return 0.0;
					}					
				}
			}
			return 1.0;
		}
*/
		return -1.0;
	}

	
	public boolean changingComponent( int iComponent, int iAction, int iObservation ){
		/*
		Action a = intToAction( iAction );
		
		if( ( a == Action.West ) || ( a == Action.East ) ){
			return iComponent == 0;
		}
		if( ( a == Action.North ) || ( a == Action.South ) ){
			return iComponent == 1;
		}
		if( a == Action.Check ){
			return false;
		}
		if( a == Action.Sample ){
			return iComponent >= 2;
		}
		*/
		return false;
	}

	
	public int[] getRelevantComponents( int iAction, int iObservation ){
		/*
		Action a = intToAction( iAction );
		if( ( a == Action.West ) || ( a == Action.East ) ){
			return new int[]{ 0 };
		}
		if( ( a == Action.North ) || ( a == Action.South ) ){
			return new int[]{ 1 };
		}
		*/
		return null;
	}

	
	public int[] getRelevantVariablesForComponent( int iAction, int iComponent ){
		/*
		Action a = intToAction( iAction );
		if( ( a == Action.West ) || ( a == Action.East ) || ( a == Action.North ) || ( a == Action.South ) ){
			return getRelevantVariables( iAction );
		}
		*/
		return null;
	}

	
	public int[] getRelevantComponentsForComponent( int iAction, int iComponent ) {
		// TODO Auto-generated method stub
		return null;
	}
		

}
