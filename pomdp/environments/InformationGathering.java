package pomdp.environments;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

public class InformationGathering extends FactoredPOMDP {

	protected int m_cMaps, m_cX, m_cY;
	protected int m_iInitialX, m_iInitialY; 
	protected int m_cXBits, m_cYBits;
	protected int[] m_aMapLocations; 
	protected double m_dObservationAccuracy;
	
	private enum Action{
		North, East, South, West;
	}

	/**
	 * Prints the rock sample map.
	 * O denotes empty square.
	 * + denotes rock location.
	 *
	 */
	public void printMap(){
		int iX = 0, iY = 0, iMap = 0;
		boolean[][] aiMap = new boolean[m_cX][m_cY];
		String sChar = "";
		for( iY = 0 ; iY < m_cY ; iY++ ){
			for( iX = 0 ; iX < m_cX ; iX++ ){
				aiMap[iX][iY] = false;	
			}
		}
		for( iMap = 0 ; iMap < m_cMaps ; iMap++ ){
			iX = m_aMapLocations[iMap];
			iY = 0;
			aiMap[iX][iY] = true;
		}
		for( iY = 0 ; iY < m_cY ; iY++ ){
			for( iX = 0 ; iX < m_cX ; iX++ ){
				if( aiMap[iX][iY] )	
					sChar = "M";
				else
					sChar = "O";
				if( ( iX == m_iInitialX ) && ( iY == m_iInitialY ) )
					sChar = "S";
				System.out.print( sChar );
			}
			System.out.println(  );
		}
	}
	
	public String getName(){
		return "InformationGathering_" + m_cMaps;
	}
			
	public InformationGathering( int cMaps, double dMapAccuracy ){
		this( cMaps, computeMapLocations( cMaps ), BeliefType.Flat, false, true, dMapAccuracy );
	}
			
	public InformationGathering( int cMaps, int[] aiMapLocations, BeliefType btFactored, boolean bUseSpecialADDForG, boolean bUseRelevantVariablesOnly, double dMapAccuracy ){
		super( 2 + cMaps + cMaps, 4, 2, btFactored, bUseSpecialADDForG, bUseRelevantVariablesOnly );
		m_cMaps = cMaps;
		m_cXBits = cMaps; 
		m_cYBits = 2;
		m_cX = (int) Math.pow( 2, cMaps );
		m_cY = 4;
		m_iInitialX = m_cX / 2; //change here to get a different starting place
		m_iInitialY = 2; //change here to get a different starting place
		m_cMaps = cMaps;
		m_cActions = 4; //north east south west 
		m_cObservations = 2; //1, 0
		m_dGamma = 0.95;
		m_aMapLocations = aiMapLocations;
		m_dObservationAccuracy = dMapAccuracy;

		initADDs();
		
		double dMdpAdr = computeMDPAverageDiscountedReward( 200, 250 );
		System.out.println( "MDP ADR = " + dMdpAdr );
		printMap();
		setObservationStates();
	}

	public boolean endADR( int iState, double dReward ){
		return dReward != 0.0;
	}
	
	
	private void setObservationStates(){
		int iMap = 0, iMapState = 0;
		int iX = 0;
		for( iMap = 0 ; iMap < m_cMaps ; iMap++ ){
			iX = m_aMapLocations[iMap];
			for( iMapState = 0 ; iMapState < Math.pow( 2, m_cMaps ) ; iMapState++ ){
				m_vObservationStates.add( stateToIndex( iX, 0, iMapState ) );
			}
		}
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
	
	protected int stateToIndex( int iX, int iY, boolean[] abMaps ){
		int iState = 0;
		int iMap = 0;
		for( iMap = m_cMaps - 1 ; iMap >= 0 ; iMap-- ){
			iState *= 2;
			if( abMaps[iMap] )
				iState += 1;
		}	
		iState = ( iState << m_cYBits ) + iY;
		iState = ( iState << m_cXBits ) + iX;
		return iState;
	}
	
	protected int stateToIndex( int iX, int iY, int iMapState ){
		int iState = iMapState;
		iState = ( iState << m_cYBits ) + iY;
		iState = ( iState << m_cXBits ) + iX;
		return iState;
	}
	
	protected int stateToIndex( boolean[] abState ){
		int iState = 0;
		int iVariable = abState.length - 1, iBit = 0;
		for( iBit = 0 ; iBit < m_cMaps ; iBit++ ){
			iState *= 2;
			if( abState[iVariable] )
				iState += 1;
			iVariable--;
		}		
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			iState *= 2;
			if( abState[iVariable] )
				iState += 1;
			iVariable--;
		}
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
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
		int iX = getX( iState ), iY = getY( iState );
		boolean[] abMaps = getMaps( iState );
		String sState = "<" + iX + "," + iY + ">,[";
		for( int iMap = 0 ; iMap < m_cMaps ; iMap++ )
			if( abMaps[iMap] )
				sState += "1";
			else
				sState += "0";
		sState += "]";
		return sState;
	}
	
	//both for transition and observation
	public int[] getRelevantVariables( int iAction ){
		/*
		int[] aiRelevant = null;
		int iStateVariable = 0;
		Action a = intToAction( iAction );
		if( a == Action.East || a == Action.West ){
			aiRelevant = new int[m_cXBits];
			for( iStateVariable = 0 ; iStateVariable < m_cXBits ; iStateVariable++ )
				aiRelevant[iStateVariable] = iStateVariable;
		}
		else if( a == Action.North || a == Action.South ){
			aiRelevant = new int[m_cYBits];
			for( iStateVariable = 0 ; iStateVariable < m_cYBits ; iStateVariable++ )
				aiRelevant[iStateVariable] = m_cXBits + iStateVariable;
		}
		return aiRelevant;
		*/
		int[] aiRelevant = new int[m_cXBits + m_cYBits + m_cMaps];
		for( int iStateVariable = 0 ; iStateVariable < m_cXBits + m_cYBits + m_cMaps ; iStateVariable++ )
			aiRelevant[iStateVariable] = iStateVariable;
		return aiRelevant;
	}

	protected int[] getObservationRelevantVariables( int iAction ){
		int[] aiRelevant = new int[m_cXBits + m_cYBits + m_cMaps];
		for( int iStateVariable = 0 ; iStateVariable < m_cXBits + m_cYBits + m_cMaps ; iStateVariable++ )
			aiRelevant[iStateVariable] = iStateVariable;
		return aiRelevant;
	}

	protected int[] getRewardRelevantVariables( int iAction ){
		int[] aiRelevant = null;
		int iStateVariable = 0;
		Action a = intToAction( iAction );
		if( a == Action.East || a == Action.West || a == Action.South ){
			aiRelevant = new int[0];
		}
		else if( a == Action.North ){
			aiRelevant = new int[getStateVariablesCount()];
			for( iStateVariable = 0 ; iStateVariable < getStateVariablesCount() ; iStateVariable++ )
				aiRelevant[iStateVariable] = iStateVariable;
		}
		return aiRelevant;
	}

	public boolean[] indexToState( int iState ){
		boolean[] abState = new boolean[getStateVariablesCount()];
		int iVariable = 0, iBit = 0;
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			abState[iVariable] = ( iState % 2 == 1 );
			iVariable++;
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			abState[iVariable] = ( iState % 2 == 1 );
			iVariable++;
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cMaps ; iBit++ ){
			abState[iVariable] = ( iState % 2 == 1 );
			iVariable++;
			iState /= 2;
		}
		
		return abState;
	}
	
	private int getX( int iState ){
		int iBit = 0, iX = 0, iPower = 1;
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			iX += ( iState % 2 ) * iPower;
			iPower *= 2;
			iState /= 2;
		}
		return iX;
	}
	
	private int getY( int iState ){
		int iBit = 0, iY = 0, iPower = 1;
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			iY += ( iState % 2 ) * iPower;
			iPower *= 2;
			iState /= 2;
		}
		return iY;
	}

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

	private boolean[] getMaps( int iState ){
		int iBit = 0;
		boolean[] abMaps = new boolean[m_cMaps];
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cMaps ; iBit++ ){
			abMaps[iBit] = ( iState % 2 == 1 );
			iState /= 2;
		}
		return abMaps;
	}
	private int getGoodDoor( int iState ){
		int iBit = 0;
		for( iBit = 0 ; iBit < m_cXBits ; iBit++ ){
			iState /= 2;
		}
		for( iBit = 0 ; iBit < m_cYBits ; iBit++ ){
			iState /= 2;
		}
		return iState;
	}
	
	private double moveTr( int iXStart, int iXEnd, int iYStart, int iYEnd, Action a ){
		if( iYStart == m_cY - 1 ){
			if( iYStart == iYEnd && iXStart == iXEnd )
				return 1.0;
			return 0.0;
		}		
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
		int iXStart = getX( iStartState ), iYStart = getY( iStartState );
		int iXEnd = getX( iEndState ), iYEnd = getY( iEndState );
		boolean[] abStartMaps = getMaps( iStartState ), abEndMaps = getMaps( iEndState );
		Action a = intToAction( iAction );
		
		if( !sameMaps( abStartMaps, abEndMaps ) ) //moves don't change Map state
			return 0.0;
			
		return moveTr( iXStart, iXEnd, iYStart, iYEnd, a ); 			
	}
	
	private boolean sameMaps( boolean[] abStartMaps, boolean[] abEndMaps ){
		int iMap = 0;
		for( iMap = 0 ; iMap < m_cMaps ; iMap++ )
			if( abStartMaps[iMap] != abEndMaps[iMap] )
				return false;
		return true;
	}

	//probability that state variable will have the value TRUE if action a is taken at state s
	protected double stateVariableTransition( int iState, int iAction, int iStateVariable ){
		Action a = intToAction( iAction );
		int iX = getX( iState );
		int iY = getY( iState );
		if( iStateVariable < m_cXBits ){ //an x coord state variable
			if( a == Action.East || a == Action.West ){ //east and west
				iX = applyXMove( iX, a ); //change the x to reflect an execution of a
			}// otherwise x stays the same
			if( ( iX >> iStateVariable ) % 2 == 1 )
				return 1.0;
			else
				return 0.0;
	
		}
		else if( iStateVariable < m_cXBits + m_cYBits ){// a y coord state variable
			if( a == Action.North || a == Action.South ){ //north and south
				iY = applyYMove( iY, a );//change the y to reflect an execution of a
			}// otherwise y stays the same
			if( ( iY >> ( iStateVariable - m_cXBits ) ) % 2 == 1 )
				return 1.0;
			else
				return 0.0;			
		}
		else if( iStateVariable < m_cXBits + m_cYBits + m_cMaps ){ // a Map bit
			boolean[] abMaps = getMaps( iState );
			int iMap = iStateVariable - m_cXBits - m_cYBits;
			if( abMaps[iMap] == true ) // Map state must not change
				return 1.0;
			else 
				return 0.0;			
		}
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
		Action a = intToAction( iAction );
		int iXStart = -1, iYStart = -1;
		int iGoodDoor = getGoodDoor( iStartState );
		if( a == Action.North ){
			iXStart = getX( iStartState );
			iYStart = getY( iStartState );
			if( iYStart == m_cY - 2 ){//about to exit
				if( iXStart == iGoodDoor )
					return 10.0; //exit through the good door
				//else
				//	return -10.0;// exit through a bad door
				// if we put a punishment identical to the reward, the agent will never learn to take it since the expected reward will (almost always) be negative
			}
		}
		return 0.0;
	}
	
	public double R( int iStartState, int iAction, int iEndState ){
		return R( iStartState, iAction ); 
	}
	
	public double R( int iStartState ){
		return -1;
	}
		
	protected int observe( int iObservation, double dProbability ){
		if( m_rndGenerator.nextDouble() < dProbability )
			return iObservation;
		else
			return 1 - iObservation;
	}
	
	public int observe( int iAction, int iEndState ){
		int iX = getX( iEndState ), iY = getY( iEndState );
		if( iY > 0 ) //not at the map row
			return 0;
		int iMap = getMapAt( iX );
		if( iMap == -1 ) // not a map square
			return 0; 
		boolean[] abMaps = getMaps( iEndState );
		int iValue = -1;
		if( abMaps[iMap] )
			iValue = 1;
		else
			iValue = 0;
		return observe( iValue, m_dObservationAccuracy );
	}
	
	
	public double O( int iAction, int iEndState, int iObservation ){
		int iX = getX( iEndState ), iY = getY( iEndState );
		if( iY > 0 ){ //not at the map row
			if( iObservation == 0 )
				return 1.0;
			else
				return 0.0;
		}
		int iMap = getMapAt( iX );
		if( iMap == -1 ){ // not a map square
			if( iObservation == 0 )
				return 1.0;
			else
				return 0.0;
		}
		boolean[] abMaps = getMaps( iEndState );
		int iValue = -1;
		if( abMaps[iMap] )
			iValue = 1;
		else
			iValue = 0;
		if( iValue == iObservation )
			return m_dObservationAccuracy;
		else
			return 1 - m_dObservationAccuracy;
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

	//begin at leftmost cloumn in the middle with at least one good Map
	public int chooseStartState(){
		int iX = m_iInitialX, iY = m_iInitialY, iMap = 0;
		boolean[] abMaps = new boolean[m_cMaps];
		for( iMap = 0 ; iMap < m_cMaps ; iMap++ ){
			if( m_rndGenerator.nextDouble() < 0.5 )
				abMaps[iMap] = false;
			else
				abMaps[iMap] = true;
		}
		
		return stateToIndex( iX, iY, abMaps );
	}

	public double probStartState( int iState ){
		int iX = getX( iState );
		int iY = getY( iState );
		if( ( iX == m_iInitialX ) && ( iY == m_iInitialY ) )
			return 1 / Math.pow( 2, m_cMaps );
		return 0.0;
	}
	
	private int getMapAt( int iX ){
		int iMap = 0;
		for( iMap = 0 ; iMap < m_cMaps ; iMap++ )
			if( m_aMapLocations[iMap] == iX )
				return iMap;
		return -1;
	}
	
	public int execute( int iAction, int iState ){
		int iX = getX( iState ), iY = getY( iState );
		boolean[] abMaps = getMaps( iState );
		Action a = intToAction( iAction );
		switch( a ){
		case North:
			if( iY < m_cY - 1 )
				iY++;
			break;
		case South:
			if( iY > 0 )
				iY--;
			break;
		case East:
			if( iX < m_cX - 1 )
				iX++;
			break;
		case West:
			if( iX > 0 )
				iX--;
			break;
		}
		int iNextState = stateToIndex( iX, iY, abMaps );
		return iNextState;
	}
	
	public int[] unExecute( int iAction, int iEndState ){
		int iX = getX( iEndState ), iY = getY( iEndState );
		boolean[] abMaps = getMaps( iEndState );
		Action a = intToAction( iAction );
		int[] aiPredeccessors = new int[1];
		switch( a ){
		case North:
			if( iY < m_cY - 1 )
				iY++;
			break;
		case South:
			if( iY > 0 )
				iY--;
			break;
		case East:
			if( iX < m_cX - 1 )
				iX++;
			break;
		case West:
			if( iX > 0 )
				iX--;
			break;
		}
		int iPreviousState = stateToIndex( iX, iY, abMaps );
		aiPredeccessors[0] = iPreviousState;
		return aiPredeccessors;
	}
	
	public boolean isTerminalState( int iState ){
		int iX = getX( iState ), iY = getY( iState );
		if( iY == m_cY - 1 )
			return true;
		return false;
	}
	
	public boolean terminalStatesDefined(){
		return true;
	}
	
	public double getMaxMinR(){
		return 0.0;
	}

	public int getStartStateCount() {
		return (int)Math.pow( 2, m_cMaps );
	}
	
	public Iterator<Entry<Integer, Double>> getStartStates() {
		int iX = m_iInitialX, iY = m_iInitialY, iMap = 0, iStartState = -1;
		int cMapStates = (int)Math.pow( 2, m_cMaps );
		double dProb = 1.0 / cMapStates;
		Map<Integer, Double> mStartStates = new TreeMap<Integer, Double>();
		for( iMap = 0 ; iMap < cMapStates ; iMap++ ){
			iStartState = stateToIndex( iX, iY, iMap );
			mStartStates.put( iStartState, dProb );
		}
		
		return mStartStates.entrySet().iterator();
	}
	
	public String getStateName( int iState ){
		int iX = getX( iState ), iY = getY( iState );
		boolean[] abMaps = getMaps( iState );
		String sState = "<x=" + iX + ",y=" + iY + ",[";
		for( boolean bMap: abMaps ){
			if( bMap )
				sState += "1";
			else
				sState += "0";
		}
		sState += "]>";
		return sState;
	}
	
	public String getActionName( int iAction ){
		String sActionName = intToAction( iAction ).toString();
		return sActionName;
	}
	
	private static int[] computeMapLocations( int cMaps ) {
		int[] aiMapLocations = new int[cMaps];
		int iMap = 0, iX = 0, iDist = (int)Math.pow( 2, cMaps ) / cMaps;
		for( iMap = 0 ; iMap < cMaps ; iMap++ ){
			aiMapLocations[iMap] = iX;
			iX += iDist;
			System.out.println( "Map " + iMap + " at " + iX );
		}
		return aiMapLocations;
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
		return "MapState";
	}

	public String getObservationName( int iObservation ) {
		if( iObservation == 1 )
			return "Good";
		else
			return "Bad";
	}

	protected String getVariableName(int iVariable) {
		if( iVariable < m_cXBits )
			return "X" + iVariable;
		else if( iVariable < m_cXBits + m_cYBits ){
			return "Y" + ( iVariable - m_cXBits );
		}
		return "Map" + ( iVariable - m_cXBits - m_cYBits );
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
		Action a = intToAction( iAction );
		if( a == Action.East || a == Action.West ){
			if( iVariable < m_cXBits ) // only X bits are relevant when moving over X axis
				return true;
		}
		else if( a == Action.North || a == Action.South ){
			if( ( iVariable >= m_cXBits ) && ( iVariable < m_cXBits + m_cYBits ) )
				return true;// only Y bits are relevant when moving over Y axis
		}
		return false;
	}

	
	private boolean getBit( int iNumber, int iDigit ){
		iNumber = iNumber >> iDigit;
		return iNumber % 2 == 1;
	}
	
	
	public double transitionGivenRelevantVariables( int iAction, int iVariable, boolean bValue, int[] aiRelevantVariables, boolean[] abValues ) {
		int iX = 0, iY = 0;
		int iMap = 0;
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
		
		return 0.0;
	}	
	
	public double observationGivenRelevantVariables( int iAction, int iObservation, int[] aiRelevantVariables, boolean[] abValues ) {
		int iX = 0, iY = 0;
		int iMap = 0;
		Action a = intToAction( iAction );
		
		if( iAction < 4 ){ //move action
			if( iObservation == 0 )
				return 1.0;
			else
				return 0.0;
		}
		
		return 0.0;
	}	
	
	public double rewardGivenRelevantVariables( int iAction, int[] aiRelevantVariables, boolean[] abValues ) {
		int iX = 0, iY = 0;
		int iMap = 0;
		Action a = intToAction( iAction );
		int iMapX = 0, iMapY = 0;
		
		
		return -100000.0;
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
		if( ( iVariable == 0 ) || ( iVariable == 1 ) ){
			if( iValue == 0 )
				return 1.0;
			else
				return 0.0;
		}
		if( ( iValue == 0 ) || ( iValue == 1 ) )
			return 0.5;
		return 0.0;
	}

	
	protected int[] getObservationRelevantVariablesMultiValue( int iAction ) {
		if( ( iAction < 4 ) || ( iAction == m_cActions - 1 ) )
			return new int[0];
		return new int[]{ 0, 1, iAction - 4 + 2 };
	}

	
	protected int getRealStateVariableCount() {
		return m_cMaps + 2;
	}

	
	protected String getRealVariableName(int iVariable) {
		if( iVariable == 0 )
			return "X";
		if( iVariable == 1 )
			return "Y";
		return "Map" + ( iVariable - 2 );
	}

	
	protected int[] getRelevantVariablesMultiValue( int iAction, int iVariable ){
		if( iAction < m_cActions - 1 ){
			return new int[]{ iVariable };
		}
		int[] aiVars = new int[m_cMaps + 2];
		for( int iVar = 0 ; iVar < aiVars.length ; iVar++ ){
			aiVars[iVar] = iVar;
		}
		return aiVars;
	}

	
	protected int getValueCount( int iVariable ) {
		if( iVariable == 0 )
			return m_cX;
		if( iVariable == 1 )
			return m_cY;
		return 2;
	}

	
	protected String getValueName( int iVariable, int iValue ){
		if( iVariable == 0 )
			return "X" + iValue;
		if( iVariable == 1 )
			return "Y" + iValue;
		if( iValue == 0)
			return "bad";
		else
			return "good";
	}

	
	protected double observationGivenRelevantVariablesMultiValue( int iAction, int iObservation, int[] aiRelevantVariables, int[] aiValues ) {
		return 0.0;
	}

	
	protected boolean relevantTransitionRealVariable( int iAction, int iVariable ) {
		return false;
	}


	
	protected double rewardGivenRelevantVariablesMultiValue( int iAction, int[] aiRelevantVariables, int[] aiValues ) {
		return -10000.0;
	}

	
	protected double transitionGivenRelevantVariablesMultiValue( int iAction, int iVariable, int iValue, int[] aiRelevantVariables, int[] aiValues ){
		return Double.POSITIVE_INFINITY;
	}

	
	protected int[] getRewardRelevantVariablesMultiValue( int iAction ){
		return null;
	}

	
	public int[] getIndependentComponentVariables( int iComponent ) {
		int[] aiVariables = null;
		return aiVariables;
	}

	
	public int getIndependentComponentsCount() {
		return m_cMaps + 2;
	}

	
	public double getInitialComponenetValueProbability( int iComponent, int iValue ){
		return 0.0;
	}
	
	
	public double transitionGivenRelevantVariables( int iAction, int[] aiComponent, boolean[] abComponentValues, 
													int[] aiRelevantVariables, boolean[] abRelevantValues ){
		return 0.0;
	}

	
	public boolean changingComponent( int iComponent, int iAction, int iObservation ){
		return false;
	}

	
	public int[] getRelevantComponents( int iAction, int iObservation ){
		return null;
	}

	
	public int[] getRelevantVariablesForComponent( int iAction, int iComponent ){
		return null;
	}

	
	public int[] getRelevantComponentsForComponent( int iAction, int iComponent ) {
		// TODO Auto-generated method stub
		return null;
	}
}

